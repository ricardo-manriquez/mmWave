from prompt_toolkit import PromptSession
from prompt_toolkit.formatted_text import HTML
from prompt_toolkit.shortcuts import message_dialog, ProgressBar
from prompt_toolkit.completion import NestedCompleter, WordCompleter, DynamicCompleter, DummyCompleter
from collections import defaultdict, OrderedDict
from inspect import signature
from mmwave import mmWave
from util import check_crc
from commands import commands
from enum import Enum
import datetime

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
        self.cmd_dict = cmd_dict
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

def main(args=None):
    b_bar = BottomBar()
    mw = mmWave(dev)
    mw.connect()
    
    rprompt = RightPrompt()
    
    session = PromptSession()
    b_bar.set_invalidate(session.app.invalidate)
    rprompt.set_invalidate(session.app.invalidate)
    
    while True:
        try: