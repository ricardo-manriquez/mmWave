import asyncio
import serial_asyncio
import struct

from prompt_toolkit.patch_stdout import patch_stdout
from prompt_toolkit.shortcuts import PromptSession
from prompt_toolkit.formatted_text import HTML
from prompt_toolkit.completion import NestedCompleter, WordCompleter, DynamicCompleter, DummyCompleter
from collections import defaultdict, OrderedDict
from inspect import signature
from enum import Enum

from util import checksum, check_pkt, fill_checksum
from commands import commands
from mmwave import mmWave

class mmWaveProtocol(asyncio.Protocol):
    def connection_made(self, transport):
        self.transport = transport
        self.m = mmWave()

    def data_received(self, data):
        if data[0:2] == b'\x53\x59':
            try:
                m_len = struct.unpack(">h", data[4:6])[0] + 1
                if check_pkt(data):
                    self.m.interpret_package(data)
            except struct.error:
                print(repr(data))
                pass

    def connection_lost(self, exc):
        print("Port closed")
        self.transport.close()

    def pause_writing(self):
        print("Pause writing")
        print(self.transport.get_write_buffer_size())

    def resume_writing(self):
        print(self.transport.get_write_buffer_size())
        print("Resume writing")

    async def execute_command(self, cmd, *args, **kwargs):
        return await self.m.execute(self.transport, cmd, *args, **kwargs)

def cmd_interpreter(cmd, result):
    if not cmd.report:
        print(cmd.name(), result)

class BottomBar:
    data = defaultdict(lambda: "")
    invalidate = None
    def update(self, cmd, data):
        #if isinstance(cmd, )
        pass
    def bottom_toolbar(self):
        return HTML(
            f'mmWave CLI - '
        )
    def set_invalidate(self, instance):
        self.invalidate = instance
    def print(self, data):
        self.data["debug"] = data
        if self.invalidate:
            self.invalidate()

class RightPrompt:
    text = ""
    def update(self, text):
        self.text = text
    def set_invalidate(self, instance):
        self.invalidate = instance
    def get(self):
        if len(self.text) > 0:
            return f'<{self.text}>'
        else:
            return ''
        if self.invalidate:
            self.invalidate()

class Autocompleter:
    def __init__(self, rprompt, cmd_dict):
        self.rprompt = rprompt
        # filter reports
        self.cmd_dict = dict()
        for cmd_name, cmd in cmd_dict.items():
            if not cmd.report:
                self.cmd_dict[cmd_name] = cmd
        self.commands_completer = WordCompleter(list(self.cmd_dict.keys()))
        self.completer = NestedCompleter.from_nested_dict({
            'help': None,
            'exec': None,
            'text': None,
            'exit': None,
            'stack': None,
            'buffer': None,
            'set_localtime': None,
            'rm': None,
            'ls': None,
            'get': None,
        })
    def autocompleter(self, document, complete_event):
        #print(document, complete_event)
        if len(document.text) == 0:
            return self.completer
        l = ' '.join(document.text.split()).split() # remove extra spaces and split
        if len(l) == 0:
            self.rprompt.update("")
        elif l[0] == "exec":
            if len(l) <= 1:
                self.rprompt.update("command")
                return self.commands_completer
            elif l[1] in self.cmd_dict:
                if len(signature(self.cmd_dict[l[1]].execute).parameters) == 0:
                    self.rprompt.update("")
                else:
                    parameters = signature(self.cmd_dict[l[1]].execute).parameters
                    if len(parameters) > (len(l) - 2):
                        param = list(parameters.items())[len(l)-2]
                        name = param[0]
                        m_type = param[1].annotation
                        self.rprompt.update(name)
                        if type(m_type) == type(Enum):
                            return WordCompleter([el.name for el in m_type])
                    else:
                        self.rprompt.update("")
        elif l[0] == "rm":
            return WordCompleter([el.name for el in FileNames])
        elif l[0] == "get":
            if len(l) > 1:
                self.rprompt.update("output filename")
                return DummyCompleter()
            return WordCompleter([el.name for el in FileNames])
        elif l[0] == "help":
            if len(l) < 2:
                self.rprompt.update("command")
                return self.commands_completer
        return DummyCompleter()

def get_completions(self, document, complete_event):
    completer = self.get_completer(document, complete_event) or DummyCompleter()
    return completer.get_completions(document, complete_event)

async def get_completions_async(self, document, complete_event):
    completer = self.get_completer(document, complete_event) or DummyCompleter()

    async for completion in completer.get_completions_async(
            document, complete_event):
        yield completion

async def interactive_shell(protocol):
    session = PromptSession("mmWave> ")
    bbar = BottomBar()
    
    type_to_command = {type(v).__name__:v for v in protocol.m.cmds.values()}
    commands_completer = WordCompleter(list(type_to_command.keys()))
    
    rprompt = RightPrompt()
    
    main_completer = Autocompleter(rprompt, type_to_command)
    
    m_completer = DynamicCompleter(main_completer.autocompleter)
    
    m_completer.get_completions = get_completions.__get__(m_completer, DynamicCompleter)
    m_completer.get_completions_async = get_completions_async.__get__(m_completer, DynamicCompleter)
    
    while True:
        try:
            result = await session.prompt_async(completer=m_completer, bottom_toolbar=bbar.bottom_toolbar)
            l = ' '.join(result.split()).split()
            if len(l) == 0:
                pass
            elif l[0] == "help":
                if len(l) == 1:
                    print("For help type help and a command name")
                else:
                    try:
                        print(type_to_command[l[1]].__doc__)
                    except KeyError:
                        print("Command not found!")
            elif l[0] == "exec":
                try:
                    cmd = type_to_command[l[1]]
                    if len(signature(cmd.execute).parameters) > 0:
                        arguments = OrderedDict()
                        try:
                            for i, (name, _class) in enumerate(signature(cmd.execute).parameters.items()):
                                if _class.annotation == Enum:
                                    arguments[name] = _class.annotation[l[i+2]]
                                elif _class.annotation == bool:
                                    arguments[name] = _class.annotation(l[i+2].lower() in ("true", "yes", "1", "y"))
                                else:
                                    arguments[name] = _class.annotation(l[i+2])
                        except ValueError as e:
                            print("Syntax Error:", e)
                        except KeyError:
                            print(f"Invalid value {e} for enumeration: {_class.annotation.__name__}")
                        except IndexError as e:
                            print("Not enough parameters\nsee: help " + type(cmd).__name__)
                        except Exception as e:
                            print("Unkown exception:", e)
                            import traceback; print(traceback.format_exc())
                        else:
                            if hasattr(cmd, "str"):
                                print(await cmd.str(protocol.execute_command(cmd.name(), **arguments)))
                            else:
                                print(await protocol.execute_command(cmd.name(), **arguments))
                    else:
                        if hasattr(cmd, "str"):
                            print(await cmd.str(protocol.execute_command(cmd.name())))
                        else:
                            print(await protocol.execute_command(cmd.name()))
                except (KeyError, IndexError):
                    print("Command not found!")
        except (EOFError, KeyboardInterrupt):
            return

async def main():
    with patch_stdout():
        loop = asyncio.get_event_loop()
        serial_task = asyncio.create_task(serial_asyncio.create_serial_connection(loop, mmWaveProtocol, "COM6", baudrate=115200))
        await serial_task
        transport, protocol = serial_task.result()
        protocol.m.set_callback(cmd_interpreter)
        try:
            await interactive_shell(protocol)
        finally:
            serial_task.cancel()
        print("Quitting event loop.")


if __name__ == "__main__":
    asyncio.run(main())