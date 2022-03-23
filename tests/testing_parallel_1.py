## Playing around with code
## Basic tests with types and abstractclass
from concurrent.futures import ThreadPoolExecutor

import numpy as np
from random import Random
from typing import List, Tuple
from abc import ABC, abstractmethod


class BaseTestClass(ABC):
    def __init__(self, operation: str ='multiply'):
        self.operation = operation

    def mtp(self, a: float = 2, b: float = 5) -> float:
        if self.operation == 'multiply':
            c = a*b
        else:
            c = a/b
        return c


    @abstractmethod
    def outra_funcao(self, a: float = 2, b: float = 5) -> float:
        pass


class TwoOperations(BaseTestClass):
    def outra_funcao(self, a: float = 2, b: float = 5) -> float:
        return a + b

mclass = TwoOperations('multiply')
res = mclass.mtp(10,2)
print(res)
res = mclass.outra_funcao(10,2)
print(res)