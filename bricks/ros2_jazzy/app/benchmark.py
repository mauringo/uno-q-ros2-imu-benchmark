"""Finite, sequential RPC -> Imu benchmark with an independent ROS receiver."""
import argparse
import csv
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import platform
import signal
import time
from datetime import datetime, timezone

from metrics import Reception, decode_packet, sustainable

TOPIC = '/benchmark/imu'


def qos(depth, reliable):
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    return QoSProfile(depth=depth, history=HistoryPolicy.KEEP_LAST,
                      reliability=(ReliabilityPolicy.RELIABLE if reliable
                                   else ReliabilityPolicy.BEST_EFFORT))


def receiver(connection, depth, reliable):
    import rclpy
    from sensor_msgs.msg import Imu
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    rclpy.init()
    node = rclpy.create_node('uno_q_speed_receiver')
    reception = None

    def callback(message):
        if reception is not None:
            value = message.linear_acceleration.x
            if math.isfinite(value) and value.is_integer():
                reception.record(int(value), time.monotonic())

    subscription = node.create_subscription(Imu, TOPIC, callback, qos(depth, reliable))
    connection.send('ready')
    try:
        while rclpy.ok():
            while connection.poll():
                command, data = connection.recv()
                if command == 'stop':
                    return
                if command == 'begin':
                    reception = Reception(**data)
                    connection.send('armed')
                elif command == 'result':
                    connection.send(reception.result(data))
                    reception = None
            rclpy.spin_once(node, timeout_sec=0.005)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
        connection.close()


def receive(connection, process, timeout=15):
    if not connection.poll(timeout):
        raise RuntimeError(f'Receiver did not answer (exit={process.exitcode})')
    return connection.recv()


def positive(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError('must be finite and positive')
    return result


def fraction(value):
    result = float(value)
    if not math.isfinite(result) or not 0 <= result <= 1:
        raise argparse.ArgumentTypeError('must be between 0 and 1')
    return result


def arguments():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--rates', default='10,25,50,75,100,150,200,400,800,1600,0',
                   help='Comma-separated target Hz; 0 is unthrottled')
    p.add_argument('--duration', type=positive, default=10.0)
    p.add_argument('--warmup', type=positive, default=1.0)
    p.add_argument('--drain', type=positive, default=1.0)
    p.add_argument('--repeats', type=int, default=3)
    p.add_argument('--rpc-timeout', type=positive, default=0.5)
    p.add_argument('--modes', nargs='+', choices=['bridge', 'ros-only'],
                   default=['bridge', 'ros-only'])
    p.add_argument('--depth', type=int, default=100)
    p.add_argument('--reliable', action='store_true')
    p.add_argument('--rate-fraction', type=fraction, default=0.95)
    p.add_argument('--max-loss', type=fraction, default=0.01)
    p.add_argument('--output', default='/results')
    args = p.parse_args()
    try:
        args.rates = [float(x) for x in args.rates.split(',')]
    except ValueError:
        p.error('--rates must be comma-separated numbers')
    if (not args.rates or any(not math.isfinite(x) or x < 0 for x in args.rates)
            or len(set(args.rates)) != len(args.rates)):
        p.error('--rates must contain unique finite nonnegative numbers')
    if args.depth < 1 or args.repeats < 1 or args.rate_fraction == 0:
        p.error('depth, repeats and rate-fraction must be positive')
    return args


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * fraction)]


def main():
    args = arguments()
    import rclpy
    from sensor_msgs.msg import Imu
    bridge = None
    if 'bridge' in args.modes:
        from app_utils import Bridge
        bridge = Bridge
    output = Path(args.output) / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    output.mkdir(parents=True, exist_ok=False)
    ctx = mp.get_context('spawn')
    parent, child = ctx.Pipe()
    process = ctx.Process(target=receiver, args=(child, args.depth, args.reliable))
    rclpy.init()
    def stop_requested(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop_requested)
    node = rclpy.create_node('uno_q_speed_publisher')
    publisher = node.create_publisher(Imu, TOPIC, qos(args.depth, args.reliable))
    message = Imu()
    message.header.frame_id = 'dummy_imu_link'
    message.orientation_covariance[0] = -1.0
    sequence = 0
    rows = []
    complete = False
    metadata = dict(config=vars(args), platform=platform.platform(),
                    ros_distro=os.getenv('ROS_DISTRO'),
                    rmw=os.getenv('RMW_IMPLEMENTATION', 'ROS default'),
                    topic=TOPIC, status='running')

    def save(status):
        metadata.update(status=status, stages=rows)
        metadata['best_sustainable_hz'] = {
            mode: max((rate for rate in args.rates if rate > 0 and
                       len([r for r in rows if r['mode'] == mode and r['target_hz'] == rate])
                       == args.repeats and all(
                           r['passed'] for r in rows
                           if r['mode'] == mode and r['target_hz'] == rate)), default=None)
            for mode in args.modes
        }
        # Replace atomically so a reader cannot observe half a summary.
        temporary = output / 'summary.tmp'
        temporary.write_text(json.dumps(metadata, indent=2) + '\n')
        temporary.replace(output / 'summary.json')

    def publish_one(mode):
        nonlocal sequence
        rpc_ms = None
        if mode == 'bridge':
            started = time.monotonic()
            axes = decode_packet(bridge.call('read_imu_fast', timeout=args.rpc_timeout))
            rpc_ms = (time.monotonic() - started) * 1000
        else:
            axes = (0.1, 0.2, 9.81, 0.01, 0.02, 0.03)
        # Sequence in one dummy axis avoids a custom ROS type and extra topic.
        # It is a host publication ID, not an MCU sampling counter.
        message.linear_acceleration.x = float(sequence)
        message.linear_acceleration.y = float(axes[1])
        message.linear_acceleration.z = float(axes[2])
        message.angular_velocity.x = float(axes[3])
        message.angular_velocity.y = float(axes[4])
        message.angular_velocity.z = float(axes[5])
        message.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(message)
        sequence += 1
        return rpc_ms

    try:
        process.start()
        child.close()
        if receive(parent, process) != 'ready':
            raise RuntimeError('Unexpected receiver startup response')
        deadline = time.monotonic() + 15
        while publisher.get_subscription_count() < 1:
            if time.monotonic() >= deadline:
                raise RuntimeError('ROS receiver discovery timed out')
            time.sleep(0.05)
        print(f'Receiver matched; results: {output}', flush=True)
        save('running')
        csv_file = output / 'stages.csv'
        for mode in args.modes:
            for repeat in range(1, args.repeats + 1):
                for rate in args.rates:
                    print(f'Start {mode} repeat={repeat} target={rate:g} Hz '
                          f'(0=unthrottled)', flush=True)
                    # Warm the whole path; fail clearly for a wrong/missing sketch.
                    warm_end = time.monotonic() + args.warmup
                    while time.monotonic() < warm_end:
                        publish_one(mode)
                    start = time.monotonic() + 0.1
                    end = start + args.duration
                    first = sequence
                    parent.send(('begin', dict(first=first, start=start, end=end)))
                    if receive(parent, process) != 'armed':
                        raise RuntimeError('Receiver did not arm')
                    # Receiver control must not consume the measurement window.
                    if time.monotonic() >= start:
                        raise RuntimeError('Receiver arm took too long; rerun on an idle board')
                    time.sleep(max(0, start - time.monotonic()))
                    next_due = start
                    rpc_times, errors, last_error = [], 0, None
                    while time.monotonic() < end:
                        now = time.monotonic()
                        if rate and now < next_due:
                            time.sleep(min(next_due - now, max(0, end - now)))
                            continue
                        if rate:
                            # Skip missed deadlines; never burst to catch up.
                            next_due = start + (int((now - start) * rate) + 1) / rate
                        try:
                            elapsed = publish_one(mode)
                            if elapsed is not None:
                                rpc_times.append(elapsed)
                        except Exception as error:
                            errors += 1
                            last_error = str(error)
                            time.sleep(min(0.01, max(0, end - time.monotonic())))
                    sent = sequence - first
                    # Include time beyond end in the report, e.g. a blocking RPC.
                    overrun = max(0, time.monotonic() - end)
                    time.sleep(args.drain)
                    parent.send(('result', sent))
                    result = receive(parent, process)
                    row = dict(mode=mode, repeat=repeat, target_hz=rate,
                               duration_s=args.duration, sent=sent,
                               sent_hz=sent / (args.duration + overrun), errors=errors,
                               last_error=last_error, overrun_ms=overrun * 1000,
                               rpc_p50_ms=percentile(rpc_times, 0.5),
                               rpc_p95_ms=percentile(rpc_times, 0.95), **result)
                    row['passed'] = sustainable(row, args.rate_fraction, args.max_loss)
                    rows.append(row)
                    with csv_file.open('a', newline='') as stream:
                        writer = csv.DictWriter(stream, fieldnames=list(row))
                        if len(rows) == 1:
                            writer.writeheader()
                        writer.writerow(row)
                    save('running')
                    print(f"  sent={row['sent_hz']:.1f} Hz received={row['received_hz']:.1f} Hz "
                          f"loss={row['loss_fraction']:.2%} errors={errors} "
                          f"pass={row['passed']}", flush=True)
        complete = True
    finally:
        save('complete' if complete else 'interrupted_or_failed')
        if process.is_alive():
            parent.send(('stop', None))
            process.join(timeout=5)
            if process.is_alive():
                process.terminate()
                process.join(timeout=5)
        parent.close()
        if bridge is not None:
            bridge.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print('Completed. Best sustainable tested target Hz (all repeats must pass):',
          metadata['best_sustainable_hz'], flush=True)
    print('Unthrottled rows show observed saturation; they do not earn a target-rate pass.', flush=True)


if __name__ == '__main__':
    main()
