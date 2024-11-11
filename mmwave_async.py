import asyncio
import serial_asyncio
import struct
from util import checksum, check_pkt, fill_checksum
from commands import commands

class mmWave:
    def __init__(self, dev):
        self.cmd_dict = {}
        self.cmds = {}
        self.dev = dev
        self.con = None
        self.callback = None
        self._filter = []
        for command in commands:
            if not command.control in self.cmd_dict:
                self.cmd_dict[command.control] = {}
            self.cmd_dict[command.control][command.command] = command()
            self.cmds[command.__name__] = self.cmd_dict[command.control][command.command]
    def set_dev(self, dev):
        self.dev = dev
    async def connect(self):
        loop = asyncio.get_event_loop()
        self.serial_task = asyncio.create_task(serial_asyncio.create_serial_connection(loop, _mmWaveProtocol, self.dev, baudrate=115200))
        await self.serial_task
        transport, protocol = self.serial_task.result()
        protocol.set_object(self)
    def add_filter(self, cmd):
        self._filter.append(type(cmd))
    def set_callback(self, callback):
        self.callback = callback
    def interpret_package(self, pkt):
        try:
            cmd = self.cmd_dict[pkt[2].to_bytes()][pkt[3].to_bytes()]
            response = self.cmd_dict[pkt[2].to_bytes()][pkt[3].to_bytes()].process(pkt)
            if self.callback:
                if self._filter:
                    if type(cmd) in self._filter:
                        self.callback(cmd, response)
                else:
                    self.callback(cmd, response)
        except KeyError as e:
            print(hex(pkt[2]), hex(pkt[3]))
            raise
        except:
            print(pkt)
            raise

class _mmWaveProtocol(asyncio.Protocol):
    def connection_made(self, transport):
        self.transport = transport

    def set_object(self, _object):
        self.m = _object

    def data_received(self, data):
        if data[0:2] == b'\x53\x59':
            m_len = struct.unpack(">h", data[4:6])[0] + 1
            if check_pkt(data):
                self.m.interpret_package(data)

    def connection_lost(self, exc):
        print("Port closed")
        self.transport.close()

    def pause_writing(self):
        print("Pause writing")
        print(self.transport.get_write_buffer_size())

    def resume_writing(self):
        print(self.transport.get_write_buffer_size())
        print("Resume writing")

    def execute_command(self, cmd):
        self.transport.write(self.m.cmds[cmd].execute())

def cmd_interpreter(cmd, result):
    print(cmd, result)


async def main():
    m = mmWave("COM6")
    await m.connect()
    m.set_callback(cmd_interpreter)
    m.add_filter(m.cmds["REPORT_HEARTRATE"])
    while True:
        await asyncio.sleep(1)
    

if __name__ == "__main__":
    asyncio.run(main())