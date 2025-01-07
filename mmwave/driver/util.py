import struct

def fill_checksum(pkt):
    l_pkt = list(pkt)
    l_pkt[-3] = checksum(pkt)
    return bytes(l_pkt)

def check_pkt(pkt):
    return pkt[-3] == checksum(pkt)

def checksum(pkt):
    m_len = struct.unpack(">h", pkt[4:6])[0] + 1
    m_sum = sum(pkt[:5+m_len])
    return (m_sum & 0xff)

class RingBuffer:
    m_len = 10
    m_buffer = bytearray(m_len)
    m_count = 0
    m_in = int()
    m_out = int()
    m_start = 0
    m_end = int()
    def insert(self, b):
        self.m_buffer[self.m_in] = int.from_bytes(b)
        self.m_in += 1
        if (self.m_in >= self.m_len):
            self.m_in = 0
        self.m_count += 1
    def get(self):
        return self.m_buffer[self.m_out]
    def getNext(self):
        if (self.m_out + 1) > self.m_len:
            return self.m_buffer[0]
        return self.m_buffer[self.m_out + 1]
    def remove(self):
        data = self.m_buffer[self.m_out]
        self.m_out += 1
        if (self.m_out >= self.m_len):
            self.m_out = 0
        self.m_count -= 1
        return data
    def isEmpty(self):
        return self.m_count == 0
    def isFull(self):
        return self.m_count == self.m_len
    def count(self):
        return self.m_count