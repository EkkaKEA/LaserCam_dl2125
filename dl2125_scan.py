# -*- coding: utf-8 -*-
"""
MV-DL2125-04H-R — скелет работы с лазерной 3D-камерой через MVS SDK (Python).

Назначение:
  * разовый скан неподвижной детали по программному триггеру;
  * сохранение кадра-профиля в PLY/CSV/OBJ (облако точек), PNG или RAW;
  * расчёт замеров (высота, объём) прямо из кадра;
  * отправка результата в ПЛК по Modbus TCP (без внешних библиотек) или TCP-сокет.

Запуск (на ПК, где установлен MVS SDK):
  python dl2125_scan.py --list
  python dl2125_scan.py --serial <SN> --count 1 --ply --png --measure

Замечания:
  * SDK ожидает ASCII-пути: не-ASCII символы в путях автоматически заменяются.
  * numpy нужен только для расчётов; без него скрипт всё равно снимает и сохраняет.
"""

import argparse
import json
import os
import re
import socket
import struct
import sys
import time
from ctypes import *

# ---------------------------------------------------------------------------
# 1. ПОДКЛЮЧЕНИЕ SDK
# ---------------------------------------------------------------------------


def add_sdk_to_path(mvimport_dir=None):
    """Добавляет каталог MvImport в sys.path.

    Ищем в таком порядке:
      1) явно переданный каталог (--mvimport);
      2) переменная окружения MVCAM_COMMON_RUNENV (её ставит установщик MVS);
      3) соседний каталог Samples/Python/MvImport (если скрипт лежит рядом с примерами).
    """
    candidates = []
    if mvimport_dir:
        candidates.append(mvimport_dir)
    runenv = os.getenv('MVCAM_COMMON_RUNENV')
    if runenv:
        candidates.append(os.path.join(runenv, 'Samples', 'Python', 'MvImport'))
    here = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.join(here, 'Samples', 'Python', 'MvImport'))
    candidates.append(os.path.join(here, 'MvImport'))
    # Скрипт лежит в подкаталоге рядом с примерами SDK: ...\камера лазерная\dl2125\...
    candidates.append(os.path.join(os.path.dirname(here), 'Samples', 'Python', 'MvImport'))

    for path in candidates:
        if os.path.isdir(path) and os.path.isfile(os.path.join(path, 'MvCameraControl_class.py')):
            if path not in sys.path:
                sys.path.insert(0, path)
            return path

    raise RuntimeError(
        'Каталог MvImport не найден. Установите MVS SDK или укажите --mvimport <путь>. '
        'Проверенные пути: ' + '; '.join(candidates)
    )


MVIMPORT_PATH = add_sdk_to_path(
    os.environ.get('DL2125_MVIMPORT') or (sys.argv[sys.argv.index('--mvimport') + 1]
                                           if '--mvimport' in sys.argv else None)
)

from MvCameraControl_class import *  # noqa: E402,F403,F401  (весь SDK-обёртка + константы)

# Строки, формируемые в Python, идут в SDK как bytes: кодировка ASCII.
SDK_ENCODING = 'ascii'

# ---------------------------------------------------------------------------
# 2. ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
# ---------------------------------------------------------------------------

# Транслитерация кириллицы: нужна, потому что SDK принимает только ASCII-пути
_CYRILLIC_TABLE = {
    u'а': 'a', u'б': 'b', u'в': 'v', u'г': 'g', u'д': 'd', u'е': 'e', u'ё': 'e',
    u'ж': 'zh', u'з': 'z', u'и': 'i', u'й': 'j', u'к': 'k', u'л': 'l', u'м': 'm',
    u'н': 'n', u'о': 'o', u'п': 'p', u'р': 'r', u'с': 's', u'т': 't', u'у': 'u',
    u'ф': 'f', u'х': 'h', u'ц': 'c', u'ч': 'ch', u'ш': 'sh', u'щ': 'sch',
    u'ъ': '', u'ы': 'y', u'ь': '', u'э': 'e', u'ю': 'yu', u'я': 'ya',
}


def translit(text):
    """Транслитерирует кириллицу в латиницу, остальное оставляет как есть."""
    out = []
    for ch in text.lower():
        for c in _CYRILLIC_TABLE.get(ch, ch):
            if ord(c) < 128:
                out.append(c)
    return ''.join(out)


def sdk_path(path):
    """Приводит путь к абсолютному ASCII-виду для SDK.

    Функции SDK (MV_CC_FeatureSave, MV_CC_SaveImageToFileEx2 и др.)
    делают encode('ascii'), поэтому кириллица в пути вызовет исключение.
    Если путь не-ASCII — кладём файл во временный каталог с транслитерированным именем.
    """
    path = os.path.abspath(path)
    try:
        path.encode(SDK_ENCODING)
        return path
    except UnicodeEncodeError:
        pass

    base = translit(os.path.basename(path)) or 'scan'
    tmp = os.path.join(os.environ.get('TEMP', '.'), 'dl2125')
    os.makedirs(tmp, exist_ok=True)
    new_path = os.path.join(tmp, base)
    print('ВНИМАНИЕ: путь "%s" содержит не-ASCII символы, файл будет сохранён как "%s"' % (path, new_path))
    return new_path


def safe_out_dir(path):
    """Создаёт каталог результатов, заменяя не-ASCII символы на транслит (ограничение SDK)."""
    path = os.path.abspath(path)
    try:
        path.encode(SDK_ENCODING)
    except UnicodeEncodeError:
        parent, base = os.path.split(path)
        new_path = os.path.join(safe_out_dir(parent), translit(base) or 'out')
        print('ВНИМАНИЕ: каталог "%s" содержит не-ASCII символы, используем "%s"' % (path, new_path))
        path = new_path
    os.makedirs(path, exist_ok=True)
    return path


def check(ret, what):
    """Проверяет код возврата SDK: MV_OK == 0, иначе бросает исключение."""
    if ret != 0:
        raise RuntimeError('%s: ошибка SDK 0x%08X' % (what, ret & 0xFFFFFFFF))


def decode_char(ctypes_char_array):
    """Декодирует C-строку из ctypes-массива (GBK/UTF-8/Latin-1)."""
    raw = memoryview(ctypes_char_array).tobytes()
    end = raw.find(b'\x00')
    if end != -1:
        raw = raw[:end]
    for enc in ('gbk', 'utf-8', 'latin-1'):
        try:
            return raw.decode(enc)
        except UnicodeDecodeError:
            continue
    return raw.decode('latin-1', errors='replace')


def import_numpy():
    """Лениво подключает numpy с понятной ошибкой, если он не установлен."""
    try:
        import numpy  # noqa: F401
        return numpy
    except ImportError:
        raise RuntimeError('Для расчётов нужен numpy: pip install numpy')


# ---------------------------------------------------------------------------
# 3. ИНИЦИАЛИЗАЦИЯ И ПЕРЕЧИСЛЕНИЕ УСТРОЙСТВ
# ---------------------------------------------------------------------------


def init_sdk():
    """Инициализирует SDK. Возвращает номер версии SDK."""
    MvCamera.MV_CC_Initialize()
    return MvCamera.MV_CC_GetSDKVersion()


def finalize_sdk():
    """Деинициализирует SDK (вызывать в finally)."""
    MvCamera.MV_CC_Finalize()


def list_devices():
    """Перечисляет доступные камеры.

    Возвращает (device_list, descriptions) — список устройств и их текстовые описания.
    Слои транспорта: GigE (сетевые) + USB3 + GenTL-варианты.
    """
    device_list = MV_CC_DEVICE_INFO_LIST()
    memset(byref(device_list), 0, sizeof(device_list))

    layer = MV_GIGE_DEVICE | MV_USB_DEVICE | MV_GENTL_GIGE_DEVICE | MV_GENTL_CAMERALINK_DEVICE
    layer |= MV_GENTL_CXP_DEVICE | MV_GENTL_XOF_DEVICE

    ret = MvCamera.MV_CC_EnumDevices(layer, device_list)
    check(ret, 'MV_CC_EnumDevices')

    descriptions = []
    for i in range(device_list.nDeviceNum):
        info = cast(device_list.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
        model = serial = ip = ''

        if info.nTLayerType in (MV_GIGE_DEVICE, MV_GENTL_GIGE_DEVICE):
            gige = info.SpecialInfo.stGigEInfo
            model = decode_char(gige.chModelName)
            serial = decode_char(gige.chSerialNumber)
            ip = '%d.%d.%d.%d' % (
                (gige.nCurrentIp >> 24) & 0xFF, (gige.nCurrentIp >> 16) & 0xFF,
                (gige.nCurrentIp >> 8) & 0xFF, gige.nCurrentIp & 0xFF)
            kind = 'GigE'
        elif info.nTLayerType == MV_USB_DEVICE:
            usb = info.SpecialInfo.stUsb3VInfo
            model = decode_char(usb.chModelName)
            serial = decode_char(usb.chSerialNumber)
            kind = 'USB3'
        else:
            kind = 'другой тип (0x%X)' % info.nTLayerType

        descriptions.append({'index': i, 'kind': kind, 'model': model, 'serial': serial, 'ip': ip})
    return device_list, descriptions


def find_device(device_list, serial=None, index=None):
    """Возвращает MV_CC_DEVICE_INFO по серийному номеру или по индексу в списке."""
    if serial:
        for i in range(device_list.nDeviceNum):
            info = cast(device_list.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
            if info.nTLayerType == MV_GIGE_DEVICE:
                sn = decode_char(info.SpecialInfo.stGigEInfo.chSerialNumber)
            elif info.nTLayerType == MV_USB_DEVICE:
                sn = decode_char(info.SpecialInfo.stUsb3VInfo.chSerialNumber)
            else:
                continue
            if sn == serial:
                return cast(device_list.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
        raise RuntimeError('Камера с серийным номером %s не найдена' % serial)

    idx = 0 if index is None else index
    if idx >= device_list.nDeviceNum:
        raise RuntimeError('Индекс %d вне диапазона (найдено устройств: %d)' % (idx, device_list.nDeviceNum))
    return cast(device_list.pDeviceInfo[idx], POINTER(MV_CC_DEVICE_INFO)).contents


# ---------------------------------------------------------------------------
# 4. ОТКРЫТИЕ / ЗАКРЫТИЕ КАМЕРЫ
# ---------------------------------------------------------------------------


def open_camera(device_info, exclusive=True):
    """Создаёт дескриптор, открывает камеру и подбирает оптимальный размер пакета GigE."""
    cam = MvCamera()

    ret = cam.MV_CC_CreateHandle(device_info)
    check(ret, 'MV_CC_CreateHandle')

    access = MV_ACCESS_Exclusive if exclusive else MV_ACCESS_Monitor
    ret = cam.MV_CC_OpenDevice(access, 0)
    if ret != 0:
        cam.MV_CC_DestroyHandle()
        check(ret, 'MV_CC_OpenDevice')

    # Печать исключений видеопотока — помогает видеть обрывы и переполнения буфера
    register_stream_exception_callback(cam)

    # Для GigE уменьшает потери пакетов на длинном кабеле/через свитч
    if device_info.nTLayerType in (MV_GIGE_DEVICE, MV_GENTL_GIGE_DEVICE):
        packet = cam.MV_CC_GetOptimalPacketSize()
        if packet > 0:
            ret = cam.MV_CC_SetIntValue('GevSCPSPacketSize', packet)
            if ret != 0:
                print('ПРЕДУПРЕЖДЕНИЕ: не удалось установить размер пакета, код 0x%X' % ret)
        else:
            print('ПРЕДУПРЕЖДЕНИЕ: не удалось получить оптимальный размер пакета')

    return cam


def close_camera(cam):
    """Останавливает съёмку (если шла), закрывает камеру и освобождает дескриптор.

    Каждый вызов защищён: повторное закрытие или уже освобождённый дескриптор
    не должны ронять процесс (нативная ошибка SDK = мгновенный вылет Python).
    """
    if cam is None or getattr(cam, '_closed', False):
        return
    cam._closed = True
    for name in ('MV_CC_StopGrabbing', 'MV_CC_CloseDevice', 'MV_CC_DestroyHandle'):
        try:
            getattr(cam, name)()
        except Exception as exc:
            print('ПРЕДУПРЕЖДЕНИЕ %s: %s' % (name, exc))


# Глобальная ссылка на колбэк исключений — иначе ctypes «соберёт» мусор и SDK упадёт
_STREAM_EXCEPTION_CB = None


def _stream_exception_callback(p_info, p_user):
    """Печатает исключения видеопотока: обрыв связи, переполнение буфера и т.п."""
    try:
        info = cast(p_info, POINTER(MV_CC_STREAM_EXCEPTION_INFO)).contents
        code = int(info.enExceptionType)
        names = {
            MV_CC_STREAM_EXCEPTION_ABNORMAL_IMAGE: 'ANOMALY_IMAGE (неправильная длина кадра)',
            MV_CC_STREAM_EXCEPTION_LIST_OVERFLOW: 'LIST_OVERFLOW (кадры не успевали забираться)',
            MV_CC_STREAM_EXCEPTION_LIST_EMPTY: 'LIST_EMPTY (кадр не возвращён вовремя)',
            MV_CC_STREAM_EXCEPTION_RECONNECTION: 'RECONNECTION (идёт восстановление связи)',
            MV_CC_STREAM_EXCEPTION_DISCONNECTED: 'DISCONNECTED (камера отвалилась)',
            MV_CC_STREAM_EXCEPTION_DEVICE: 'DEVICE (устройство не в норме)',
            MV_CC_STREAM_EXCEPTION_PARTIAL_IMAGE: 'PARTIAL_IMAGE (неполный кадр отброшен)',
            MV_CC_STREAM_EXCEPTION_IMAGE_BUFFER_OVERFLOW: 'BUFFER_OVERFLOW (кадр больше буфера)',
        }
        print('ИСКЛЮЧЕНИЕ ВИДЕОПОТОКА: 0x%X %s' % (code, names.get(code, '')))
    except Exception:
        pass


def register_stream_exception_callback(cam):
    """Подключает обработчик исключений видеопотока (для диагностики обрывов).

    Вызываем DLL напрямую с правильными argtypes: обёртка задаёт c_void_p и
    ctypes не умеет превратить функцию в такой аргумент.
    """
    global _STREAM_EXCEPTION_CB
    try:
        fun_ctype = get_platform_functype()
        # argtypes нужен СОБРАННЫЙ тип колбэка (у него есть from_param),
        # а не фабрика CFUNCTYPE — иначе ctypes откажется преобразовывать
        cb_type = fun_ctype(None, POINTER(MV_CC_STREAM_EXCEPTION_INFO), c_void_p)
        _STREAM_EXCEPTION_CB = cb_type(_stream_exception_callback)
        dll = MvCamCtrldll
        dll.MV_CC_RegisterExceptionCallBack.argtypes = (c_void_p, cb_type, c_void_p)
        dll.MV_CC_RegisterExceptionCallBack.restype = c_uint
        dll.MV_CC_RegisterExceptionCallBack(cam.handle, _STREAM_EXCEPTION_CB, None)
    except Exception as exc:
        print('ПРЕДУПРЕЖДЕНИЕ: не удалось зарегистрировать обработчик исключений: %s' % exc)


# ---------------------------------------------------------------------------
# 5. ПАРАМЕТРЫ КАМЕРЫ
# ---------------------------------------------------------------------------


def get_payload_size(cam):
    """Размер кадра в байтах (полезная нагрузка), который придёт из камеры."""
    st = MVCC_INTVALUE()
    memset(byref(st), 0, sizeof(st))
    check(cam.MV_CC_GetIntValue('PayloadSize', st), 'MV_CC_GetIntValue(PayloadSize)')
    return st.nCurValue


def get_image_size(cam):
    """Текущие Width/Height кадра из GenICam-параметров камеры."""
    w = MVCC_INTVALUE()
    h = MVCC_INTVALUE()
    memset(byref(w), 0, sizeof(w))
    memset(byref(h), 0, sizeof(h))
    check(cam.MV_CC_GetIntValue('Width', w), 'MV_CC_GetIntValue(Width)')
    check(cam.MV_CC_GetIntValue('Height', h), 'MV_CC_GetIntValue(Height)')
    return w.nCurValue, h.nCurValue


def set_exposure_time(cam, exposure_us):
    """Экспозиция в микросекундах (для лазерного профиля подбирается под отражение лазера)."""
    if exposure_us is None:
        return
    check(cam.MV_CC_SetFloatValue('ExposureTime', float(exposure_us)), 'MV_CC_SetFloatValue(ExposureTime)')


def set_gain(cam, gain):
    """Усиление (аналоговая/цифровая нормализация). None — не трогаем."""
    if gain is None:
        return
    check(cam.MV_CC_SetFloatValue('Gain', float(gain)), 'MV_CC_SetFloatValue(Gain)')


def set_pixel_format(cam, pixel_format_value):
    """Переключает формат пикселей (значения PixelType_Gvsp_*). None — не трогаем."""
    if pixel_format_value is None:
        return
    check(cam.MV_CC_SetEnumValue('PixelFormat', pixel_format_value), 'MV_CC_SetEnumValue(PixelFormat)')


# Режимы вывода кадров, найденные в дампе параметров этой камеры (узел ImageMode):
#   Origin      — исходное 2D-изображение (снимок лазерной полосы, Mono8);
#   PointCloud3D — кадр с 3D-данными ( Coord3D_* ), из него строится облако точек;
#   Range3D     — карта высот;
#   PkgMask     — маска упакованного объекта (логистика DWS).
IMAGE_MODES = ('Origin', 'PointCloud3D', 'Range3D', 'PkgMask')


def set_image_mode(cam, mode):
    """Переключает режим вывода кадров камеры (узел ImageMode).

    Это главный узел 3D-режима: в Origin камера отдаёт Mono8 (полосу лазера),
    в PointCloud3D/Range3D — данные высот.
    """
    if mode not in IMAGE_MODES:
        raise RuntimeError('Допустимые режимы ImageMode: %s' % ', '.join(IMAGE_MODES))
    check(cam.MV_CC_SetEnumValueByString('ImageMode', mode), 'MV_CC_SetEnumValueByString(ImageMode=%s)' % mode)
    print('ImageMode установлен: %s' % mode)


def enable_chunk(cam, selector, enable=True):
    """Включает/выключает дополнительные данные (chunk), прикладываемые к кадру.

    В дампе камеры доступные селекторы: Image, Thickness (толщина), RunDir.
    После включения в кадре появляются блоки с числовыми значениями.
    """
    check(cam.MV_CC_SetEnumValueByString('ChunkSelector', selector),
          'MV_CC_SetEnumValueByString(ChunkSelector=%s)' % selector)
    check(cam.MV_CC_SetBoolValue('ChunkEnable', enable), 'MV_CC_SetBoolValue(ChunkEnable)')
    print('Chunk %s: %s' % (selector, 'включён' if enable else 'выключен'))


def enable_event(cam, name='DwsVolume'):
    """Включает событие камеры (в дампе доступно DwsVolume — «объём посчитан»).

    Важно: по документации SDK полезная нагрузка события (pEventData) прошивкой
    не заполняется, поэтому сам объём из события прочитать нельзя — это лишь сигнал
    «результат готов». Числа берём из кадра (measure_object) либо из TCP-выхода камеры.
    """
    check(cam.MV_CC_EventNotificationOn(), 'MV_CC_EventNotificationOn')
    check(cam.MV_CC_SetEnumValueByString('EventSelector', name),
          'MV_CC_SetEnumValueByString(EventSelector=%s)' % name)
    print('Событие включено: %s' % name)


def get_pixel_format(cam):
    """Текущий формат пикселей кадра (значение PixelType_Gvsp_*)."""
    st = MVCC_ENUMVALUE()
    memset(byref(st), 0, sizeof(st))
    check(cam.MV_CC_GetEnumValue('PixelFormat', st), 'MV_CC_GetEnumValue(PixelFormat)')
    return st.nCurValue


def save_camera_params(cam, path):
    """Сохраняет ВСЕ параметры камеры (включая калибровку) в файл .txt."""
    path = sdk_path(path)
    check(cam.MV_CC_FeatureSave(path), 'MV_CC_FeatureSave(%s)' % path)
    print('Параметры камеры сохранены: %s' % path)
    return path


def load_camera_params(cam, path):
    """Загружает параметры камеры (включая калибровку) из файла .txt."""
    path = sdk_path(path)
    check(cam.MV_CC_FeatureLoad(path), 'MV_CC_FeatureLoad(%s)' % path)
    print('Параметры камеры загружены: %s' % path)


# Ключевые слова, по которым ищем узлы 3D-режима в дампе параметров камеры
PARAM_KEYWORDS = (
    '3d', 'height', 'laser', 'profile', 'coord', 'volume', 'point', 'cloud',
    'zmap', 'depth', 'scan', 'line', 'alg', 'measure', 'output', 'format',
    'mode', 'unit', 'dws', 'lsl', 'range', 'tcp', 'event',
)


def dump_camera_params(cam, path=None, show_all=False):
    """Сохраняет полный список параметров камеры и печатает узлы, связанные с 3D.

    MV_CC_FeatureSave выгружает ВСЮ дерево GenICam камеры — это самый быстрый
    способ узнать, каким узлом включается вывод высот/облака точек.
    """
    if path is None:
        path = os.path.join(os.environ.get('TEMP', '.'), 'dl2125_params.txt')
    path = sdk_path(path)
    check(cam.MV_CC_FeatureSave(path), 'MV_CC_FeatureSave(%s)' % path)

    raw = open(path, 'rb').read()
    text = None
    for enc in ('utf-8', 'gbk', 'latin-1'):
        try:
            text = raw.decode(enc)
            break
        except UnicodeDecodeError:
            continue
    if text is None:
        text = raw.decode('latin-1', errors='replace')

    # Формат дампа: GenApi persistence — строки "Имя<TAB>значение" или "Имя = значение"
    # (комментарии с # пропускаются); запасной вариант — XML с Name="ИмяУзла"
    pairs = []
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        m = re.match(r'^([A-Za-z][A-Za-z0-9_]*)\s*(?:=|\t)\s*(.*?)\s*$', line)
        if m:
            pairs.append((m.group(1), m.group(2)))
    if not pairs:
        for name in re.findall(r'Name="([^"]+)"', text):
            pairs.append((name, ''))
    names = [n for n, _ in pairs]

    print('Параметров в камере: %d, дамп записан в %s' % (len(names), path))
    matched = [(n, v) for n, v in pairs if any(k in n.lower() for k in PARAM_KEYWORDS)]
    print('Узлы с ключевыми словами (%s): %d' % (', '.join(PARAM_KEYWORDS[:7]) + '...', len(matched)))
    for n, v in matched:
        print('  %s = %s' % (n, v))
    if show_all:
        print('--- все узлы ---')
        for n, v in pairs:
            print('  %s = %s' % (n, v))
    return names, text


def _wait_file_access(cam, timeout_s=30.0):
    """Ожидает завершения асинхронной операции доступа к файлу камеры.

    SDK запускает чтение/запись файла в отдельном потоке, поэтому после
    MV_CC_FileAccessRead/Write нужно опрашивать прогресс, пока всё не перенесётся.
    """
    progress = MV_CC_FILE_ACCESS_PROGRESS()
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        ret = cam.MV_CC_GetFileAccessProgress(progress)
        if ret != 0:
            return ret
        if progress.nTotal > 0 and progress.nCompleted == progress.nTotal:
            return 0
        time.sleep(0.05)
    return -1  # таймаут ожидания


def read_camera_file(cam, camera_filename, host_path, timeout_s=30.0):
    """Читает файл из памяти камеры (например, калибровочные данные) на диск.

    camera_filename — имя файла В камере (например 'UserSet1');
    host_path       — путь на ПК, куда сохранить результат.
    """
    access = MV_CC_FILE_ACCESS()
    memset(byref(access), 0, sizeof(access))
    host_path = sdk_path(host_path)
    access.pUserFileName = host_path.encode(SDK_ENCODING)
    access.pDevFileName = camera_filename.encode(SDK_ENCODING)

    ret = cam.MV_CC_FileAccessRead(access)
    check(ret, 'MV_CC_FileAccessRead(%s)' % camera_filename)
    ret = _wait_file_access(cam, timeout_s)
    if ret != 0:
        raise RuntimeError('Ожидание чтения файла из камеры не завершено, код 0x%X' % (ret & 0xFFFFFFFF))
    print('Файл "%s" из камеры записан в %s (%d байт)'
          % (camera_filename, host_path, os.path.getsize(host_path)))


def write_camera_file(cam, host_path, camera_filename, timeout_s=30.0):
    """Записывает файл с ПК в память камеры."""
    if not os.path.isfile(host_path):
        raise RuntimeError('Файл %s не найден' % host_path)

    access = MV_CC_FILE_ACCESS()
    memset(byref(access), 0, sizeof(access))
    access.pUserFileName = sdk_path(host_path).encode(SDK_ENCODING)
    access.pDevFileName = camera_filename.encode(SDK_ENCODING)

    ret = cam.MV_CC_FileAccessWrite(access)
    check(ret, 'MV_CC_FileAccessWrite(%s)' % camera_filename)
    ret = _wait_file_access(cam, timeout_s)
    if ret != 0:
        raise RuntimeError('Ожидание записи файла в камеру не завершено, код 0x%X' % (ret & 0xFFFFFFFF))
    print('Файл %s записан в камеру как "%s"' % (host_path, camera_filename))


# ---------------------------------------------------------------------------
# 6. ТРИГГЕР
# ---------------------------------------------------------------------------


def set_trigger_off(cam):
    """Непрерывная съёмка: камера сама шлёт кадры (бесполезно для статичной детали, но годится для отладки)."""
    check(cam.MV_CC_SetEnumValue('TriggerMode', MV_TRIGGER_MODE_OFF), 'MV_CC_SetEnumValue(TriggerMode=Off)')


def set_software_trigger(cam, trigger_source='Software', trigger_selector='FrameBurstStart'):
    """Программный триггер: кадр снимается командой fire_software_trigger().

    Это базовый сценарий для неподвижной детали: деталь поставили — нажали «скан».
    В дампе этой камеры селектор триггера умеет значения FrameBurstStart и LineStart.
    """
    # Порядок важен: сначала селектор, затем режим, затем источник
    cam.MV_CC_SetEnumValueByString('TriggerSelector', trigger_selector)
    check(cam.MV_CC_SetEnumValue('TriggerMode', MV_TRIGGER_MODE_ON), 'MV_CC_SetEnumValue(TriggerMode=On)')
    if trigger_source == 'Software':
        source = MV_TRIGGER_SOURCE_SOFTWARE
    elif trigger_source == 'Line0':
        source = MV_TRIGGER_SOURCE_LINE0
    else:
        raise RuntimeError('Неизвестный источник триггера: %s' % trigger_source)
    check(cam.MV_CC_SetEnumValue('TriggerSource', source), 'MV_CC_SetEnumValue(TriggerSource)')
    print('Триггер: %s / %s' % (trigger_selector, trigger_source))


def set_burst_count(cam, count):
    """Задаёт число кадров в серии на один TriggerSoftware.

    Камера в 3D-режиме собирает облако из нескольких лазерных линий; если кадр один
    не приходит, пробуем серию (AcquisitionBurstFrameCount).
    """
    if count is None:
        return
    check(cam.MV_CC_SetIntValue('AcquisitionBurstFrameCount', int(count)),
          'MV_CC_SetIntValue(AcquisitionBurstFrameCount)')
    print('AcquisitionBurstFrameCount = %d' % count)


def fire_trigger_sequence(cam, retriggers=0, delay_s=0.002):
    """Отдаёт TriggerSoftware (плюс несколько повторов с паузой) до забора кадра."""
    fire_software_trigger(cam)
    for _ in range(max(0, retriggers)):
        time.sleep(delay_s)
        fire_software_trigger(cam)


def fire_software_trigger(cam):
    """Отдаёт команду «TriggerSoftware» — камера начинает экспозицию одного кадра."""
    check(cam.MV_CC_SetCommandValue('TriggerSoftware'), 'MV_CC_SetCommandValue(TriggerSoftware)')


# ---------------------------------------------------------------------------
# 7. СЪЁМКА КАДРА
# ---------------------------------------------------------------------------


def start_grabbing(cam):
    """Запускает трансляцию кадров из камеры."""
    check(cam.MV_CC_StartGrabbing(), 'MV_CC_StartGrabbing')


def stop_grabbing(cam):
    """Останавливает трансляцию кадров."""
    check(cam.MV_CC_StopGrabbing(), 'MV_CC_StopGrabbing')


def read_chunks(frame_struct):
    """Считывает chunk-данные кадра (пока буфер SDK ещё не освобождён).

    Возвращает список словарей {'id': int, 'data': bytes}.
    Защищено от мусорных значений: нативное обращение по неверному указателю
    мгновенно роняет процесс, поэтому все поля проверяются.
    """
    chunks = []
    try:
        count = int(frame_struct.stFrameInfo.nUnparsedChunkNum)
        if count <= 0 or count > 32:  # реальные chunk-блоки — единицы, не сотни
            return chunks
        head = frame_struct.stFrameInfo.UnparsedChunkList.pUnparsedChunkContent
        if not head:
            return chunks
        for i in range(count):
            item = cast(head, POINTER(MV_CHUNK_DATA_CONTENT))[i]
            length = int(item.nChunkLen)
            if length <= 0 or length > (1 << 20) or not item.pChunkData:
                continue
            chunks.append({'id': int(item.nChunkID), 'data': string_at(item.pChunkData, length)})
    except Exception as exc:
        print('ПРЕДУПРЕЖДЕНИЕ: chunk-данные не прочитаны: %s' % exc)
    return chunks


def describe_chunks(chunks):
    """Печатает содержимое chunk-блоков: id, длина и числовая интерпретация.

    Для маленьких блоков (толщина и т.п.) пробуем разобрать как float32/int32
    little-endian — так значение сразу видно в консоли.
    """
    if not chunks:
        return
    for ch in chunks:
        data = ch['data']
        line = '  chunk id=0x%04X, %d байт' % (ch['id'], len(data))
        if 0 < len(data) <= 32:
            if len(data) % 4 == 0:
                floats = struct.unpack('<%df' % (len(data) // 4), data)
                ints = struct.unpack('<%di' % (len(data) // 4), data)
                line += ' | float: %s | int: %s' % (
                    ', '.join('%.4f' % v for v in floats), ', '.join(str(v) for v in ints))
            else:
                line += ' | hex: %s' % data.hex()
        print(line)


def grab_frame(cam, timeout_ms=5000):
    """Забирает один кадр и возвращает его копию в виде словаря.

    Возвращает:
      {
        'data': bytes,          # сырые байты кадра (копия, буфер SDK освобождён)
        'width': int,
        'height': int,
        'pixel_type': int,      # значение PixelType_Gvsp_*
        'frame_num': int,
        'timestamp_ms': int,    # отметка времени на хосте
      }
    """
    frame = MV_FRAME_OUT()
    memset(byref(frame), 0, sizeof(frame))

    ret = cam.MV_CC_GetImageBuffer(frame, timeout_ms)
    if ret != 0 or frame.pBufAddr is None:
        raise RuntimeError('Кадр не получен (таймаут или ошибка) 0x%X' % (ret & 0xFFFFFFFF))

    try:
        width = frame.stFrameInfo.nExtendWidth or frame.stFrameInfo.nWidth
        height = frame.stFrameInfo.nExtendHeight or frame.stFrameInfo.nHeight
        length = frame.stFrameInfo.nFrameLenEx or frame.stFrameInfo.nFrameLen
        # Chunk-данные читаем здесь: после FreeImageBuffer указатели станут недействительны
        chunks = read_chunks(frame)
        # ВАЖНО: делаем копию, т.к. после FreeImageBuffer память возвращается в пул SDK
        data = string_at(frame.pBufAddr, length)
        return {
            'data': data,
            'width': int(width),
            'height': int(height),
            'pixel_type': int(frame.stFrameInfo.enPixelType),
            'frame_num': int(frame.stFrameInfo.nFrameNum),
            'timestamp_ms': int(frame.stFrameInfo.nHostTimeStamp),
            'chunks': chunks,
        }
    finally:
        cam.MV_CC_FreeImageBuffer(frame)


def scan_once(cam, timeout_ms=5000, retriggers=0):
    """Одна полная операция: программный триггер + захват одного кадра.

    retriggers>0 — после первого TriggerSoftware отдать ещё несколько (камеры,
    собирающие 3D из многих линий, не отдают кадр после одной линии).
    """
    fire_trigger_sequence(cam, retriggers=retriggers)
    return grab_frame(cam, timeout_ms)


# ---------------------------------------------------------------------------
# 8. РАЗБОР КАДРА ( Coord3D_* -> numpy )
# ---------------------------------------------------------------------------

# Поддерживаемые форматы: 'имя' -> (PixelType-константа, код numpy, байт на пиксель)
FORMAT_TABLE = None  # заполняется лениво, т.к. константы приходят из SDK


def _format_table():
    """Таблица форматов: (имя, код dtype numpy, байт на пиксель).

    Коды — строки ('u1', 'f4'), чтобы отчёт о кадре работал БЕЗ numpy;
    numpy нужен только для расчётов (frame_to_array/measure).
    """
    global FORMAT_TABLE
    if FORMAT_TABLE is None:
        FORMAT_TABLE = {
            PixelType_Gvsp_Mono8: ('Mono8', 'u1', 1),
            PixelType_Gvsp_Mono16: ('Mono16', 'u2', 2),
            # C16 этой камеры — ЗНАКОВЫЕ высоты int16 (сырое 0x8000 = -32768 = «нет данных»)
            PixelType_Gvsp_Coord3D_C16: ('Coord3D_C16', 'i2', 2),
            PixelType_Gvsp_Coord3D_C32: ('Coord3D_C32', 'i4', 4),
            # C32f — высота в мм с плавающей точкой
            PixelType_Gvsp_Coord3D_C32f: ('Coord3D_C32f', 'f4', 4),
            # ABC32f — по три float на пиксель: X, Y, Z
            PixelType_Gvsp_Coord3D_ABC32f: ('Coord3D_ABC32f', 'f4', 12),
            # AC32f — два float на пиксель: интенсивность и Z
            PixelType_Gvsp_Coord3D_AC32f: ('Coord3D_AC32f', 'f4', 8),
        }
    return FORMAT_TABLE


def frame_info(frame):
    """Человекочитаемое описание кадра (формат, размер, число пикселей)."""
    table = _format_table()
    name, dtype, bpp = table.get(frame['pixel_type'], ('НЕИЗВЕСТНЫЙ', None, None))
    return {
        'format': name,
        'dtype': dtype,
        'width': frame['width'],
        'height': frame['height'],
        'pixel_type_hex': '0x%08X' % frame['pixel_type'],
        'bytes': len(frame['data']),
        'known': dtype is not None,
        'bytes_per_pixel': bpp,
    }


def frame_to_array(frame):
    """Преобразует байты кадра в numpy-массив (height, width) или (height, width, 3).

    Для Coord3D_ABC32f возвращается массив (H, W, 3) с X, Y, Z.
    Для остальных — двумерный массив высот/яркости.
    """
    np = import_numpy()
    info = frame_info(frame)
    if not info['known']:
        raise RuntimeError(
            'Формат пикселей %s не встроен в таблицу форматов. '
            'Добавьте его в _format_table() или сохраните кадр через save_raw().' % info['pixel_type_hex'])

    dtype = np.dtype(info['dtype'])
    arr = np.frombuffer(frame['data'], dtype=dtype, count=len(frame['data']) // dtype.itemsize)
    if info['format'] == 'Coord3D_ABC32f':
        return arr.reshape((frame['height'], frame['width'], 3)).copy()
    if info['format'] == 'Coord3D_AC32f':
        return arr.reshape((frame['height'], frame['width'], 2)).copy()
    return arr.reshape((frame['height'], frame['width'])).copy()


# ---------------------------------------------------------------------------
# 9. СОХРАНЕНИЕ КАДРА
# ---------------------------------------------------------------------------


def frame_points_mm(frame, x_mm_per_px=0.1, y_mm_per_px=0.1, z_scale=1000.0):
    """Список точек (x_мм, y_мм, z_мм) для валидных пикселей кадра.

    Поддерживаемые форматы:
      Coord3D_C16  — профиль высот, ЗНАКОВЫЕ int16 в единицах SDK
                     (сырое 0x8000 = -32768 = «нет данных», 0 = нет отражения);
      Coord3D_C32f — высота в мм с плавающей точкой;
      Coord3D_ABC32f — готовые координаты X, Y, Z (калиброванные, мм).
    Невалидные точки (0 / -32768 / NaN) пропускаются.
    Работает без numpy — на чистом Python (array).
    """
    import array as _array
    width = frame['width']
    pixel_type = frame['pixel_type']
    pts = []
    if pixel_type == PixelType_Gvsp_Coord3D_C16:
        values = _array.array('h')  # int16 little-endian — значения знаковые
        values.frombytes(frame['data'])
        for idx, value in enumerate(values):
            # 0 — нет отражения; -32768 (сырое 0x8000) — маркер «нет данных»
            if value in (0, -32768):
                continue
            row, col = divmod(idx, width)
            pts.append((col * x_mm_per_px, row * y_mm_per_px, value / z_scale))
    elif pixel_type == PixelType_Gvsp_Coord3D_C32f:
        values = _array.array('f')
        values.frombytes(frame['data'])
        for idx, value in enumerate(values):
            if value != value:  # NaN
                continue
            row, col = divmod(idx, width)
            pts.append((col * x_mm_per_px, row * y_mm_per_px, value))
    elif pixel_type == PixelType_Gvsp_Coord3D_ABC32f:
        values = _array.array('f')
        values.frombytes(frame['data'])
        for i in range(0, len(values) - 2, 3):
            x, y, z = values[i], values[i + 1], values[i + 2]
            if z != z or z == 0.0:
                continue
            pts.append((x, y, z))
    else:
        raise RuntimeError(
            'Для формата 0x%08X облако точек не строится. Нужен Coord3D_C16/C32f/ABC32f — '
            'снимайте с --image-mode PointCloud3D' % pixel_type)
    return pts


def save_point_cloud(frame, path, fmt='ply', x_mm_per_px=0.1, y_mm_per_px=0.1, z_scale=1000.0):
    """Сохраняет кадр-профиль как облако точек: PLY / CSV / OBJ.

    Пишется на хосте (без MV_CC_SavePointCloudData): SDK этой камеры отвергает
    Coord3D_C16 с ошибкой MV_E_PARAMETER, а свой писатель работает для всех
    поддерживаемых форматов и не требует numpy.
    """
    import array as _array
    fmt = fmt.lower()
    if fmt not in ('ply', 'csv', 'obj'):
        raise RuntimeError('Поддерживаемые форматы: ply, csv, obj')
    ext = '.' + fmt
    if not path.lower().endswith(ext):
        path += ext
    path = sdk_path(path)

    pts = frame_points_mm(frame, x_mm_per_px, y_mm_per_px, z_scale)
    if not pts:
        raise RuntimeError('В кадре нет валидных точек (все невалидные/нулевые)')

    if fmt == 'ply':
        header = ('ply\nformat binary_little_endian 1.0\n'
                  'comment units: mm (x,y from pixel size, z from z_scale)\n'
                  'element vertex %d\nproperty float x\nproperty float y\nproperty float z\n'
                  'end_header\n' % len(pts))
        buf = _array.array('f')
        for x, y, z in pts:
            buf.extend((x, y, z))
        with open(path, 'wb') as f:
            f.write(header.encode('ascii'))
            buf.tofile(f)
    elif fmt == 'csv':
        with open(path, 'w', encoding='ascii') as f:
            f.write('x_mm;y_mm;z_mm\n')
            for x, y, z in pts:
                f.write('%.4f;%.4f;%.4f\n' % (x, y, z))
    else:  # obj
        with open(path, 'w', encoding='ascii') as f:
            f.write('# MV-DL2125 point cloud, units: mm\n')
            for x, y, z in pts:
                f.write('v %.4f %.4f %.4f\n' % (x, y, z))

    print('Облако точек сохранено: %s (%d точек, %s)' % (path, len(pts), fmt))
    return path


def save_image(cam, frame, path, fmt='png'):
    """Сохраняет кадр как обычное изображение (png/bmp/jpg/tif) силами SDK."""
    kind = {'png': MV_Image_Png, 'bmp': MV_Image_Bmp, 'jpg': MV_Image_Jpeg, 'tif': MV_Image_Tif}
    fmt = fmt.lower()
    if fmt not in kind:
        raise RuntimeError('Поддерживаемые форматы изображений: png, bmp, jpg, tif')
    if not path.lower().endswith('.' + fmt):
        path += '.' + fmt
    path = sdk_path(path)

    image = MV_CC_IMAGE()
    memset(byref(image), 0, sizeof(image))
    image.nWidth = frame['width']
    image.nHeight = frame['height']
    image.enPixelType = frame['pixel_type']
    # Копия байтов: ссылка img_buf должна жить до конца вызова SDK
    img_buf = (c_ubyte * len(frame['data'])).from_buffer_copy(frame['data'])
    image.pImageBuf = cast(img_buf, POINTER(c_ubyte))
    image.nImageBufSize = len(frame['data'])
    image.nImageLen = len(frame['data'])

    param = MV_CC_SAVE_IMAGE_PARAM()
    memset(byref(param), 0, sizeof(param))
    param.enImageType = kind[fmt]
    param.iMethodValue = 1
    param.nQuality = 90

    check(cam.MV_CC_SaveImageToFileEx2(image, param, path), 'MV_CC_SaveImageToFileEx2(%s)' % path)
    print('Изображение сохранено: %s' % path)
    return path


def save_raw(frame, path):
    """Сохраняет сырые байты кадра + JSON-описание.

    Используется, когда формат пикселей неизвестен или SDK не умеет писать его в PLY.
    """
    path = sdk_path(path)
    raw_path = os.path.splitext(path)[0] + '.raw'
    meta_path = os.path.splitext(path)[0] + '.json'

    with open(raw_path, 'wb') as f:
        f.write(frame['data'])

    meta = frame_info(frame)
    meta.update({'frame_num': frame['frame_num'], 'timestamp_ms': frame['timestamp_ms']})
    with open(meta_path, 'w', encoding='utf-8') as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)

    print('RAW сохранён: %s (+ %s)' % (raw_path, meta_path))
    return raw_path


# ---------------------------------------------------------------------------
# 10. ЗАМЕРЫ И ОБЪЁМ
# ---------------------------------------------------------------------------


def measure_object(frame, z_scale=1000.0, x_mm_per_px=0.1, y_mm_per_px=0.1,
                   invalid_values=(0,), base_mm=None):
    """Считает замеры по кадру-профилю.

    Параметры калибровки:
      z_scale     — сколько единиц SDK приходится на 1 мм по оси Z
                    (Coord3D_C16: знаковые единицы измерителя — калибруется по
                     известному размеру объекта; Coord3D_C32f в мм — 1);
      x_mm_per_px — размер пикселя по поперечной оси, мм;
      y_mm_per_px — шаг развёртки (или размер пикселя по продольной оси), мм;
      base_mm     — уровень опорной плоскости в мм. None — принять минимальную
                    валидную высоту за базу (плоскость стола/подложки вокруг детали).

    Возвращает словарь с результатами; поля с 'None' означают «нет данных».
    """
    np = import_numpy()
    arr = frame_to_array(frame)

    # Для ABC32f берём третью компоненту — это высота Z
    if arr.ndim == 3 and arr.shape[2] == 3:
        z = arr[:, :, 2].astype(np.float64)
    else:
        z = arr.astype(np.float64)

    # Маскируем «пустые» точки.
    # Coord3D_C16 этой камеры: значения знаковые int16;
    # -32768 (сырое 0x8000, 83% кадра!) = маркер «нет данных» измерителя — это НЕ высота;
    # 0 = лазер не вернул отражение; для float-форматов — только NaN и 0.
    invalid_values = tuple(sorted(set(tuple(invalid_values) + (-32768, 32768))))
    mask = np.ones(z.shape, dtype=bool)
    for bad in invalid_values:
        if bad is None:
            mask &= ~np.isnan(z)
        else:
            mask &= (z != bad)
    mask &= ~np.isnan(z)

    valid = int(mask.sum())
    if valid == 0:
        return {'valid_points': 0, 'note': 'нет валидных точек — проверьте фокус/экспозицию/отражение лазера'}

    z_valid = z[mask]
    z_top = float(z_valid.max())

    # Опорная плоскость: явно заданная (base_mm) или минимальная по валидным точкам
    if base_mm is None:
        z_base = float(z_valid.min())
    else:
        z_base = float(base_mm) * z_scale

    # Объём «над базой»: сумма высот * площадь одного пикселя
    cell_area = x_mm_per_px * y_mm_per_px
    volume_mm3 = float(np.maximum(z_valid - z_base, 0).sum()) * cell_area / z_scale

    # Габариты в миллиметрах
    height_mm = max(z_top - z_base, 0.0) / z_scale
    bbox_x_mm = frame['width'] * x_mm_per_px
    bbox_y_mm = frame['height'] * y_mm_per_px

    return {
        'valid_points': valid,
        'total_points': int(z.size),
        'fill_ratio': valid / float(z.size),
        'z_base_raw': z_base,
        'z_max_raw': z_top,
        'height_mm': height_mm,
        'volume_mm3': volume_mm3,
        'volume_cm3': volume_mm3 / 1000.0,
        'bbox_x_mm': bbox_x_mm,
        'bbox_y_mm': bbox_y_mm,
    }


# ---------------------------------------------------------------------------
# 11. ВЫДАЧА РЕЗУЛЬТАТА В ПЛК / SCADA
# ---------------------------------------------------------------------------


def values_to_modbus_registers(values):
    """Упаковывает список вещественных чисел в Modbus-регистры (по 2 регистра на float32, big-endian)."""
    data = b''.join(struct.pack('>f', float(v)) for v in values)
    return list(struct.unpack('>%dH' % (len(data) // 2), data))


def modbus_write_floats(host, port, unit_id, start_address, values, timeout=2.0):
    """Записывает вещественные числа в holding-регистры ПЛК по Modbus TCP.

    Реализован на голых сокетах, чтобы не тянуть pymodbus.
    Функция 0x10 (Write Multiple Registers), по 2 регистра на float32 (IEEE754, старший регистр первым).
    """
    regs = values_to_modbus_registers(values)
    if len(regs) > 123:
        raise RuntimeError('За одну операцию Modbus можно записать не более 123 регистров, т.е. 61 float')

    transaction_id = int(time.time() * 1000) & 0xFFFF
    pdu = struct.pack('>BHHB', 0x10, start_address, len(regs), len(regs) * 2)
    pdu += struct.pack('>%dH' % len(regs), *regs)
    mbap = struct.pack('>HHHB', transaction_id, 0, len(pdu) + 1, unit_id)

    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.sendall(mbap + pdu)
        resp = sock.recv(256)

    if len(resp) < 8:
        raise RuntimeError('Короткий ответ Modbus: %r' % resp)
    # В успешном ответе функция 0x10 (бит 7 не установлен)
    func = resp[7]
    if func & 0x80:
        code = resp[8] if len(resp) > 8 else 0
        raise RuntimeError('Modbus-ошибка, код исключения 0x%02X' % code)
    return True


def send_values_tcp_json(host, port, payload, timeout=2.0):
    """Отправляет результат одной JSON-строкой по TCP (удобно для SCADA/своего приёмника)."""
    line = (json.dumps(payload, ensure_ascii=False) + '\n').encode('utf-8')
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.sendall(line)
    return True


def publish_results(results, args):
    """Отправляет результаты измерения наружу: Modbus TCP и/или TCP+JSON."""
    if not args.modbus_host and not args.json_host:
        return

    # Порядок регистров должен совпадать с адресацией на стороне ПЛК
    floats = [
        results.get('height_mm') or 0.0,
        results.get('volume_cm3') or 0.0,
        results.get('valid_points') or 0.0,
        results.get('fill_ratio') or 0.0,
    ]

    if args.modbus_host:
        try:
            modbus_write_floats(args.modbus_host, args.modbus_port, args.modbus_unit,
                                args.modbus_addr, floats)
            print('Отправлено в ПЛК (Modbus TCP %s:%d, unit %d, адрес %d): %s'
                  % (args.modbus_host, args.modbus_port, args.modbus_unit, args.modbus_addr,
                     ', '.join('%.3f' % v for v in floats)))
        except Exception as exc:
            print('ПРЕДУПРЕЖДЕНИЕ: не удалось отправить в ПЛК: %s' % exc)

    if args.json_host:
        try:
            send_values_tcp_json(args.json_host, args.json_port, results)
            print('Отправлено по TCP+JSON на %s:%d' % (args.json_host, args.json_port))
        except Exception as exc:
            print('ПРЕДУПРЕЖДЕНИЕ: не удалось отправить по TCP: %s' % exc)


# ---------------------------------------------------------------------------
# 12. ОСНОВНОЙ ЦИКЛ
# ---------------------------------------------------------------------------


def print_scan_report(index, info, results=None):
    """Печатает итог одного скана в консоль. results=None — только описание кадра."""
    print('--- скан #%d ---' % index)
    print('кадр %d x %d, формат %s (%s), байт %d'
          % (info['width'], info['height'], info['format'], info['pixel_type_hex'], info['bytes']))
    if results is None:
        return
    if results.get('valid_points') == 0:
        print('РЕЗУЛЬТАТ: %s' % results.get('note'))
        return
    print('валидных точек: %d из %d (заполнение %.1f%%)'
          % (results['valid_points'], results['total_points'], results['fill_ratio'] * 100))
    print('высота: %.3f мм' % results['height_mm'])
    print('объём:  %.3f см3' % results['volume_cm3'])
    print('сырые единицы Z: база %.0f, макс %.0f (для калибровки --z-scale)'
          % (results['z_base_raw'], results['z_max_raw']))
    print('габариты поля: %.1f x %.1f мм' % (results['bbox_x_mm'], results['bbox_y_mm']))


def run_scan(args):
    """Основной рабочий цикл: открыть камеру -> N сканов -> закрыть."""
    sdk_ver = init_sdk()
    print('SDK инициализирован, версия 0x%08X' % sdk_ver)
    cam = None
    try:
        device_list, devices = list_devices()
        if not devices:
            print('Камеры не найдены. Проверьте: кабель/сеть, IP-подсеть, брандмауэр Windows, '
                  'что камера не занята другой программой (MVS/3DMVS).')
            return 1

        print('Найдено устройств: %d' % len(devices))
        for dev in devices:
            print('  [%d] %s  модель=%s  серийник=%s  IP=%s'
                  % (dev['index'], dev['kind'], dev['model'], dev['serial'], dev['ip']))

        if args.list:
            return 0

        device_info = find_device(device_list, serial=args.serial, index=args.index)
        cam = open_camera(device_info)

        # Дамп дерева параметров — отдельный режим диагностики, съёмку не ведём
        if args.dump_params is not None:
            dump_camera_params(cam, args.dump_params or None, show_all=args.dump_all)
            print('Дамп параметров завершён. Для съёмки уберите --dump-params.')
            return 0

        # Режим кадров и дополнительные данные — до съёмки, иначе формат останется Mono8
        if args.image_mode:
            set_image_mode(cam, args.image_mode)
        for selector in args.chunk:
            enable_chunk(cam, selector)
        if args.event:
            enable_event(cam, args.event)

        # --- настройки перед съёмкой ---
        set_exposure_time(cam, args.exposure)
        set_gain(cam, args.gain)
        set_pixel_format(cam, args.pixel_format)
        if args.params_load:
            load_camera_params(cam, args.params_load)

        # Состояние печатаем после всех переключений: формат мог смениться
        w_cur, h_cur = get_image_size(cam)
        print('Параметры съёмки: поле %d x %d, PixelFormat=0x%08X, PayloadSize=%d'
              % (w_cur, h_cur, get_pixel_format(cam), get_payload_size(cam)))

        # После смены режима видеопоток GigE может остаться «старым»: переоткрываем
        # камеру, чтобы SDK заново зарегистрировал канал потока под новый payload.
        if args.reopen:
            print('Переоткрытие камеры для перерегистрации видеопотока...')
            close_camera(cam)
            cam = open_camera(device_info)
            print('Камера переоткрыта: PixelFormat=0x%08X, PayloadSize=%d'
                  % (get_pixel_format(cam), get_payload_size(cam)))

        if args.free_run:
            set_trigger_off(cam)      # непрерывный поток кадров
        else:
            set_software_trigger(cam, trigger_selector=args.trigger_selector)
            set_burst_count(cam, args.burst)

        args.out = safe_out_dir(args.out)
        start_grabbing(cam)
        print('Съёмка запущена, каталог результатов: %s' % args.out)

        try:
            for i in range(args.count):
                if args.free_run:
                    frame = grab_frame(cam, args.timeout)
                else:
                    frame = scan_once(cam, args.timeout, retriggers=args.retriggers)

                info = frame_info(frame)
                stamp = time.strftime('%Y%m%d_%H%M%S') + '_%04d' % (i + 1)
                base = os.path.join(args.out, 'scan_' + stamp)

                if frame.get('chunks'):
                    print('chunk-данные кадра:')
                    describe_chunks(frame['chunks'])

                if args.raw or not info['known']:
                    try:
                        save_raw(frame, base)
                    except Exception as exc:
                        print('ПРЕДУПРЕЖДЕНИЕ: не удалось сохранить RAW: %s' % exc)

                if info['known']:
                    for flag, kind_name in ((args.ply, 'ply'), (args.csv, 'csv'), (args.obj, 'obj')):
                        if not flag:
                            continue
                        # Ошибка сохранения одного формата не должна ронять весь скан
                        try:
                            save_point_cloud(frame, base, kind_name,
                                             x_mm_per_px=args.x_mm_per_px,
                                             y_mm_per_px=args.y_mm_per_px,
                                             z_scale=args.z_scale)
                        except Exception as exc:
                            print('ПРЕДУПРЕЖДЕНИЕ: не удалось сохранить %s: %s' % (kind_name, exc))

                    if args.png:
                        if info['format'] in ('Mono8', 'Mono16'):
                            try:
                                save_image(cam, frame, base, 'png')
                            except Exception as exc:
                                print('ПРЕДУПРЕЖДЕНИЕ: не удалось сохранить png: %s' % exc)
                        else:
                            print('PNG пропущен: формат %s — не изображение. Для PNG переключите режим: '
                                  '--image-mode Origin' % info['format'])

                    if args.measure:
                        try:
                            results = measure_object(
                                frame,
                                z_scale=args.z_scale,
                                x_mm_per_px=args.x_mm_per_px,
                                y_mm_per_px=args.y_mm_per_px,
                                base_mm=args.base_mm,
                            )
                        except Exception as exc:
                            print('ПРЕДУПРЕЖДЕНИЕ: расчёт не удался: %s' % exc)
                        else:
                            print_scan_report(i + 1, info, results)
                            publish_results(results, args)
                    else:
                        print_scan_report(i + 1, info)
                else:
                    print('Формат %s не распознан — кадр сохранён только в RAW' % info['pixel_type_hex'])

                if i + 1 < args.count:
                    time.sleep(args.interval)
        finally:
            stop_grabbing(cam)
            print('Съёмка остановлена')

        if args.params_save:
            save_camera_params(cam, args.params_save)

        return 0
    finally:
        close_camera(cam)
        finalize_sdk()


def build_parser():
    """Описание аргументов командной строки."""
    p = argparse.ArgumentParser(description='MV-DL2125-04H-R: скан профиля, замеры, выдача в ПЛК')

    p.add_argument('--list', action='store_true', help='только перечислить камеры и выйти')
    p.add_argument('--serial', default=None, help='серийный номер камеры (если их несколько)')
    p.add_argument('--index', type=int, default=0, help='индекс камеры в списке (по умолчанию 0)')
    p.add_argument('--mvimport', default=None, help='путь к каталогу MvImport SDK, если нет MVCAM_COMMON_RUNENV')

    p.add_argument('--count', type=int, default=1, help='число сканов (по умолчанию 1)')
    p.add_argument('--interval', type=float, default=1.0, help='пауза между сканами, с')
    p.add_argument('--timeout', type=int, default=5000, help='таймаут ожидания кадра, мс')
    p.add_argument('--free-run', action='store_true', help='непрерывная съёмка вместо программного триггера')

    p.add_argument('--exposure', type=float, default=None, help='экспозиция, мкс')
    p.add_argument('--gain', type=float, default=None, help='усиление')
    p.add_argument('--pixel-format', type=int, default=None,
                   help='формат пикселей (значение PixelType_Gvsp_*, например 17825976 = Coord3D_C16)')
    p.add_argument('--image-mode', default=None, choices=list(IMAGE_MODES),
                   help='режим вывода кадров: Origin (2D), PointCloud3D, Range3D, PkgMask')
    p.add_argument('--chunk', action='append', default=[], choices=['Image', 'Thickness', 'RunDir'],
                   help='включить chunk-данные к кадру (можно указать несколько раз)')
    p.add_argument('--event', default=None, help='включить событие камеры, например DwsVolume')
    p.add_argument('--trigger-selector', default='FrameBurstStart', choices=['FrameBurstStart', 'LineStart'],
                   help='селектор триггера (по умолчанию FrameBurstStart)')
    p.add_argument('--burst', type=int, default=None,
                   help='AcquisitionBurstFrameCount: сколько кадров отдаёт камера на один TriggerSoftware')
    p.add_argument('--retriggers', type=int, default=0,
                   help='дополнительные TriggerSoftware перед забором кадра (серия линий для сборки 3D)')
    p.add_argument('--reopen', action='store_true',
                   help='после смены режима переоткрыть камеру (перерегистрация видеопотока GigE)')

    p.add_argument('--out', default='./out', help='каталог для результатов')
    p.add_argument('--ply', action='store_true', help='сохранять облако точек в PLY')
    p.add_argument('--csv', action='store_true', help='сохранять облако точек в CSV')
    p.add_argument('--obj', action='store_true', help='сохранять облако точек в OBJ')
    p.add_argument('--png', action='store_true', help='сохранять кадр как изображение')
    p.add_argument('--raw', action='store_true', help='сохранять сырые байты кадра + JSON')

    p.add_argument('--measure', action='store_true', help='считать высоту и объём')
    p.add_argument('--z-scale', type=float, default=1000.0,
                   help='единиц SDK на 1 мм по Z (знаковые единицы C16 — калибруются, обычно 1000; 1 для мм)')
    p.add_argument('--x-mm-per-px', type=float, default=0.1, help='размер пикселя по X, мм')
    p.add_argument('--y-mm-per-px', type=float, default=0.1, help='размер пикселя по Y, мм')
    p.add_argument('--base-mm', type=float, default=None,
                   help='уровень опорной плоскости в мм (по умолчанию — минимум валидных точек)')

    p.add_argument('--params-save', default=None, help='сохранить все параметры камеры в файл')
    p.add_argument('--params-load', default=None, help='загрузить параметры камеры из файла')
    p.add_argument('--dump-params', nargs='?', const='', default=None, metavar='ФАЙЛ',
                   help='выгрузить дерево параметров камеры (для поиска узла 3D-режима) и выйти')
    p.add_argument('--dump-all', action='store_true', help='вместе с --dump-params печатать все узлы')

    p.add_argument('--modbus-host', default=None, help='IP ПЛК для Modbus TCP')
    p.add_argument('--modbus-port', type=int, default=502, help='порт Modbus TCP')
    p.add_argument('--modbus-unit', type=int, default=1, help='unit id (slave) Modbus')
    p.add_argument('--modbus-addr', type=int, default=0, help='адрес первого holding-регистра')

    p.add_argument('--json-host', default=None, help='IP приёмника для TCP+JSON')
    p.add_argument('--json-port', type=int, default=10000, help='порт приёмника TCP+JSON')
    return p


def main():
    args = build_parser().parse_args()
    # Без явного выбора форматов сохраняем хотя бы PLY — иначе скан без следа
    if not any([args.ply, args.csv, args.obj, args.png, args.raw]):
        args.ply = True
        args.png = True
    try:
        return run_scan(args)
    except Exception as exc:
        print('ОШИБКА: %s' % exc)
        return 2


if __name__ == '__main__':
    sys.exit(main())
