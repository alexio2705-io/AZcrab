"""Импорт модуля робота без железа: тяжёлые зависимости (state тянет numpy,
lidar_walls - порт лидара, leds - spidev) подменяются пустыми модулями
только на время импорта. Модуль потом видит подставные объекты как свои
глобальные имена - их и настраивает тест."""
import importlib
import os
import sys
import types
from unittest import mock

PI_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PI_DIR not in sys.path:
    sys.path.insert(0, PI_DIR)


def fake_module(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    return m


def import_with(module_name, **fakes):
    """Импортировать module_name, подменив модули fakes (имя -> модуль).
    После импорта sys.modules возвращается как был."""
    with mock.patch.dict(sys.modules, fakes):
        sys.modules.pop(module_name, None)
        return importlib.import_module(module_name)
