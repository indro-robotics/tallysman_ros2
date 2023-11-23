
from enum import Enum
from typing import Literal


class ByteType(Enum):
    U8 = 0,
    U16 = 1,
    U32 = 2,
    U64 = 3,
    S8 = 10,
    S16 = 11,
    S32 = 12,
    S64 = 13,

class Prs:
    """
    Converts the no of bytes to int based on bytetype.
    returns a tuple -- (result, updated_ix)
    """
    @staticmethod
    def ConvertToInt(buf: bytearray, ix: int, bit_type: ByteType, byte_order: Literal['little', 'big'] = 'little'):
        if bit_type == ByteType.S8:
            return int.from_bytes(buf[ix: ix+1], byte_order, signed=True), ix + 1
        elif bit_type == ByteType.S16:
            return int.from_bytes(buf[ix: ix+2], byte_order, signed=True), ix + 2
        elif bit_type == ByteType.S32:
            return int.from_bytes(buf[ix: ix+3], byte_order, signed=True), ix + 3
        elif bit_type == ByteType.S64:
            return int.from_bytes(buf[ix: ix+4], byte_order, signed=True), ix + 4
        elif bit_type == ByteType.U8:
            return int.from_bytes(buf[ix: ix+1], byte_order, signed=False), ix + 1
        elif bit_type == ByteType.U16:
            return int.from_bytes(buf[ix: ix+2], byte_order, signed=False), ix + 2
        elif bit_type == ByteType.U32:
            return int.from_bytes(buf[ix: ix+3], byte_order, signed=False), ix + 3
        elif bit_type == ByteType.U64:
            return int.from_bytes(buf[ix: ix+4], byte_order, signed=False), ix + 4
        return 0,ix
    
    """\
    skips the reserved bits of length <len> from index <ix>. returns updated index.
    """
    @staticmethod
    def Reserved(buf: bytearray, ix: int, len: int):
        return ix + len
    
    """\
    Extracts <len> bytes from bytearray starting from index <ix>.
    returns a tuple -- (result, updated_ix)
    """
    @staticmethod
    def Bytes(buf: bytearray, ix: int, len: int):
        return buf[ix: ix+len], ix+len
    
    """\
    Converts specific bits in a byte to int.
    returns a int
    """
    @staticmethod
    def ConvertBitsToInt(byte: int, pos: int, numberOfBits: int):
        return ((byte>>pos) & Prs.GetMaskInt(numberOfBits))
    
    """\
    returns a masked int for extracting bits of specified length.
    returns a tuple -- (result, updated_ix)
    """
    @staticmethod
    def GetMaskInt(len: int):
        return (2**len) - 1
    
class UbloxUtility:
    @staticmethod
    def Checksum(buf: bytearray, startIx: int, len: int):
        ckA: int = 0
        ckB: int = 0
        for val in buf[startIx: startIx+len]:
            ckA += val
            ckA = ckA & Prs.GetMaskInt(8)
            ckB += ckA
            ckB = ckB & Prs.GetMaskInt(8)
        return (ckA + (ckB << 8))