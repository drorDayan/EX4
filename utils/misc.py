def between(s, p, f):
    return (s <= p <= f) or (f <= p <= s)


def interweave(l1, l2):
    return [m for pair in zip(l1, l2) for m in pair]