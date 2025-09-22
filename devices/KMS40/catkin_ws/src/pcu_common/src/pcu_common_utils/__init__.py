import random, string

def genId():
    return ''.join(random.SystemRandom().choice(string.ascii_uppercase + string.digits) for _ in range(20))
