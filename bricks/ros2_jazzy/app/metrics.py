"""ROS-independent packet validation and throughput accounting."""
import math


def decode_packet(packet):
    if not isinstance(packet, (list, tuple)) or len(packet) != 8:
        raise ValueError("Expected eight-float dummy IMU reply")
    if any(isinstance(x, bool) or not isinstance(x, (int, float))
           or not math.isfinite(x) for x in packet):
        raise ValueError("Packet must contain finite numbers")
    if packet[0] != 1 or packet[1] != 0:
        raise ValueError("Deploy the speed-test sketch (ready=1, rate=0)")
    return packet[2:]


class Reception:
    def __init__(self, first, start, end):
        self.first, self.start, self.end = first, start, end
        self.seen = set()
        self.in_window = 0
        self.duplicates = 0
        self.reordered = 0
        self.highest = first - 1

    def record(self, sequence, now):
        if sequence < self.first or now < self.start:
            return
        if sequence in self.seen:
            self.duplicates += 1
            return
        self.seen.add(sequence)
        if sequence < self.highest:
            self.reordered += 1
        self.highest = max(self.highest, sequence)
        if now <= self.end:
            self.in_window += 1

    def result(self, sent):
        # No later stage starts until this result has been collected.
        received = sum(self.first <= s < self.first + sent for s in self.seen)
        return dict(received=received, received_in_window=self.in_window,
                    duplicates=self.duplicates, reordered=self.reordered,
                    lost=max(0, sent - received),
                    loss_fraction=(sent - received) / sent if sent else 1.0,
                    received_hz=self.in_window / (self.end - self.start))


def sustainable(row, rate_fraction, max_loss):
    return (row['target_hz'] > 0 and row['sent'] > 0 and row['errors'] == 0
            and row['sent_hz'] >= row['target_hz'] * rate_fraction
            and row['received_hz'] >= row['target_hz'] * rate_fraction
            and row['loss_fraction'] <= max_loss)
