import serial
import struct
import asyncio
from util import checksum, check_pkt, fill_checksum
from commands import commands

class mmWave():
    def __init__(self):
        self.cmd_dict = {}
        self.cmds = {}
        self.dev = None
        self.con = None
        self.callback = None
        self._filter = []
        self.stack = {}
        for command in commands:
            if not command.control in self.cmd_dict:
                self.cmd_dict[command.control] = {}
            self.cmd_dict[command.control][command.command] = command()
            self.cmds[command.__name__] = self.cmd_dict[command.control][command.command]
    def set_dev(self, dev):
        self.dev = dev
    def connect(self):
        self.con = serial.Serial(self.dev, 115200)
    def set_callback(self, callback):
        self.callback = callback
    def add_filter(self, cmd):
        self._filter.append(type(cmd))
    async def execute(self, transport, cmd, *args, **kwargs):
        transport.write(self.cmds[cmd].execute(*args, **kwargs))
        if hasattr(self.cmds[cmd], "process"):
            self.stack[cmd] = asyncio.Event()
            try:
                await asyncio.wait_for(self.stack[cmd].wait(), 2.0)
            except TimeoutError:
                print("Command timed out")
                raise TimeoutError("Command exceeded timeout.")
            else:
                return self.stack[cmd]
    def interpret_package(self, pkt):
        try:
            cmd = self.cmd_dict[pkt[2].to_bytes()][pkt[3].to_bytes()]
            response = None
            if cmd.name() in self.stack:
                self.stack[cmd.name()].set()
                self.stack[cmd.name()] = cmd.process(pkt)
            else:
                if hasattr(cmd, "process"):
                    response = cmd.process(pkt)
                else:
                    response = repr(pkt)
            if self.callback:
                if self._filter:
                    if type(cmd) in self._filter:
                        if response:
                            self.callback(cmd, response)
                else:
                    if response:
                        self.callback(cmd, response)
        except:
            print(pkt)
            raise

def cmd_interpreter(cmd, result):
    print(cmd.name(), result)


def main(arg):

    if len(arg) < 2:
        print(f"Usage: {arg[0]} com_port")
        import sys; sys.exit(1)

    a = serial.Serial(arg[1], 115200)
    
    m = mmWave()
    
    #a.write(m.cmds["CMD_QUERY_HUMAN_PRESENCE_SWITCH"].execute())
    m.set_callback(cmd_interpreter)
    
    m.add_filter(m.cmds["REPORT_HEARTRATE"])
    
    while True:
        b = a.read(6)
        if b[0:2] == b'\x53\x59':
            m_len = struct.unpack(">h", b[4:6])[0] + 1
            b += a.read(m_len + 2)
            if check_pkt(b):
                m.interpret_package(b)
        else:
            b = a.read_until(b"\x54\x43")
    a.close()

if __name__ == "__main__":
    import sys
    main(sys.argv)