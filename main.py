# PicoAWG v1.0
# ref: Rolf Oldeman's Arbitrary waveform generator
# https://www.instructables.com/Arbitrary-Wave-Generator-With-the-Raspberry-Pi-Pic/
# tested with rp2-pico-20230426-v1.20.0.uf2

from machine import Pin, mem32, PWM
from rp2 import PIO, StateMachine, asm_pio
from array import array
from math import pi, sin, exp, sqrt, floor
from uctypes import addressof
from random import random
from micropython import const
#################################SETTINGS#######################################

fclock = const(250000000)  # clock frequency of the pico
dac_clock = int(fclock/2)
# make buffers for the waveform.
# large buffers give better results but are slower to fill
maxnsamp = const(4096)  # must be a multiple of 4. miximum size is 65536
wavbuf = {}
wavbuf[0] = bytearray(maxnsamp*2)
wavbuf[1] = bytearray(maxnsamp*2)
ibuf = 0
temp = None
################################################################################


DMA_BASE = const(0x50000000)
CH0_READ_ADDR = const(DMA_BASE+0x000)
CH0_WRITE_ADDR = const(DMA_BASE+0x004)
CH0_TRANS_COUNT = const(DMA_BASE+0x008)
CH0_CTRL_TRIG = const(DMA_BASE+0x00c)
CH0_AL1_CTRL = const(DMA_BASE+0x010)
CH1_READ_ADDR = const(DMA_BASE+0x040)
CH1_WRITE_ADDR = const(DMA_BASE+0x044)
CH1_TRANS_COUNT = const(DMA_BASE+0x048)
CH1_CTRL_TRIG = const(DMA_BASE+0x04c)
CH1_AL1_CTRL = const(DMA_BASE+0x050)

PIO0_BASE = const(0x50200000)
PIO0_TXF0 = const(PIO0_BASE+0x10)
PIO0_SM0_CLKDIV = const(PIO0_BASE+0xc8)

# set desired clock frequency
machine.freq(fclock)

# set default DAC current to 10mA
dac904_bias = PWM(Pin(14))
dac904_bias.freq(100000)
dac904_bias.duty_u16(32768)

# state machine that just pushes bytes to the pins


@asm_pio(sideset_init=(PIO.OUT_HIGH,),
         out_init=(PIO.OUT_HIGH,)*14,
         out_shiftdir=PIO.SHIFT_RIGHT,
         fifo_join=PIO.JOIN_TX,
         autopull=True,
         pull_thresh=28)
def stream():
    wrap_target()
    out(pins, 14) .side(0)
    nop() 		.side(1)
    wrap()


sm = StateMachine(0, stream, freq=fclock,
                  sideset_base=Pin(15), out_base=Pin(0))
sm.active(1)

# 2-channel chained DMA. channel 0 does the transfer, channel 1 reconfigures
p = array('I', [0])  # global 1-element array


def startDMA(ar, nword):
    # first disable the DMAs to prevent corruption while writing
    mem32[CH0_AL1_CTRL] = 0
    mem32[CH1_AL1_CTRL] = 0
    # setup first DMA which does the actual transfer
    mem32[CH0_READ_ADDR] = addressof(ar)
    mem32[CH0_WRITE_ADDR] = PIO0_TXF0
    mem32[CH0_TRANS_COUNT] = nword
    IRQ_QUIET = 0x1  # do not generate an interrupt
    TREQ_SEL = 0x00  # wait for PIO0_TX0
    CHAIN_TO = 1  # start channel 1 when done
    RING_SEL = 0
    RING_SIZE = 0  # no wrapping
    INCR_WRITE = 0  # for write to array
    INCR_READ = 1  # for read from array
    DATA_SIZE = 2  # 32-bit word transfer
    HIGH_PRIORITY = 1
    EN = 1
    CTRL0 = (IRQ_QUIET << 21) | (TREQ_SEL << 15) | (CHAIN_TO << 11) | (RING_SEL << 10) | (RING_SIZE << 9) | (
        INCR_WRITE << 5) | (INCR_READ << 4) | (DATA_SIZE << 2) | (HIGH_PRIORITY << 1) | (EN << 0)
    mem32[CH0_AL1_CTRL] = CTRL0
    # setup second DMA which reconfigures the first channel
    p[0] = addressof(ar)
    mem32[CH1_READ_ADDR] = addressof(p)
    mem32[CH1_WRITE_ADDR] = CH0_READ_ADDR
    mem32[CH1_TRANS_COUNT] = 1
    IRQ_QUIET = 0x1  # do not generate an interrupt
    TREQ_SEL = 0x3f  # no pacing
    CHAIN_TO = 0  # start channel 0 when done
    RING_SEL = 0
    RING_SIZE = 0  # no wrapping
    INCR_WRITE = 0  # single write
    INCR_READ = 0  # single read
    DATA_SIZE = 2  # 32-bit word transfer
    HIGH_PRIORITY = 1
    EN = 1
    CTRL1 = (IRQ_QUIET << 21) | (TREQ_SEL << 15) | (CHAIN_TO << 11) | (RING_SEL << 10) | (RING_SIZE << 9) | (
        INCR_WRITE << 5) | (INCR_READ << 4) | (DATA_SIZE << 2) | (HIGH_PRIORITY << 1) | (EN << 0)
    mem32[CH1_CTRL_TRIG] = CTRL1


def setupwave(buf, f, w):
    # required clock division for maximum buffer size
    div = dac_clock/(f*maxnsamp)
    if div < 1.0:  # can't speed up clock, duplicate wave instead
        dup = int(1.0/div)
        nsamp = int((maxnsamp*div*dup+0.5)/4)*4  # force multiple of 4
        clkdiv = 1
    else:  # stick with integer clock division only
        clkdiv = int(div)+1
        nsamp = int((maxnsamp*div/clkdiv+0.5)/4)*4  # force multiple of 4
        dup = 1
    # gain set
    if w.amplitude >= 2:
        dac904_bias.duty_u16(65535)
        w.amplitude = 0.5
        w.offset = 0
    elif w.amplitude <= 0.2:
        if w.offset == 0:
            dac904_bias.duty_u16(6554)
            w.amplitude = w.amplitude/0.2*0.5
        else:
            dac904_bias.duty_u16(65535)
            w.amplitude = w.amplitude/2*0.5
            w.offset = w.offset/2
    else:
        k = 1-w.offset/w.amplitude
        dac904_bias.duty_u16(int(w.amplitude/2/k*65535))
        w.amplitude = 0.5*k
        w.offset = 0.5-0.5*k

    # fill the buffer
    # for isamp in range(nsamp):
    #     buf[isamp] = max(0, min(255, int(255*eval(w, dup*(isamp+0.5)/nsamp))))

    # print([dup, clkdiv, nsamp, int(nsamp/2)])
    if w.func is not None:
        for iword in range(int(nsamp/2)):
            val1 = int(16383*eval(w, dup*(iword*2+0)/nsamp))+8192
            val2 = int(16383*eval(w, dup*(iword*2+1)/nsamp))+8192

            word = val1 + (val2 << 14)
            buf[iword*4+0] = (word & (255 << 0)) >> 0
            buf[iword*4+1] = (word & (255 << 8)) >> 8
            buf[iword*4+2] = (word & (255 << 16)) >> 16
            buf[iword*4+3] = (word & (255 << 24)) >> 24
    # set the clock divider
    clkdiv_int = min(clkdiv, 65535)
    clkdiv_frac = 0  # fractional clock division results in jitter
    mem32[PIO0_SM0_CLKDIV] = (clkdiv_int << 16) | (clkdiv_frac << 8)

    # start DMA
    startDMA(buf, int(nsamp/2))


# evaluate the content of a wave
def eval(w, x):
    m, s, p = 1.0, 0.0, 0.0
    if 'phasemod' in w.__dict__:
        p = eval(w.phasemod, x)
    if 'mult' in w.__dict__:
        m = eval(w.mult, x)
    if 'sum' in w.__dict__:
        s = eval(w.sum, x)
    x = x*w.replicate-w.phase-p
    x = x-floor(x)  # reduce x to 0.0-1.0 range
    v = w.func(x, w.pars)
    v = v*w.amplitude*m
    v = v+w.offset+s
    return v

# some common waveforms. combine with sum,mult,phasemod


def sine(x, pars):
    return sin(x*2*pi)


def pulse(x, pars):  # risetime,uptime,falltime
    if x < pars[0]:
        return -1+2*x/pars[0]
    if x < pars[0]+pars[1]:
        return 1.0
    if x < pars[0]+pars[1]+pars[2]:
        return -1+2*(1.0-(x-pars[0]-pars[1])/pars[2])
    return -1


def gaussian(x, pars):
    return exp(-((x-0.5)/pars[0])**2)


def sinc(x, pars):
    if x == 0.5:
        return 1.0
    else:
        return sin((x-0.5)/pars[0])/((x-0.5)/pars[0])


def exponential(x, pars):
    return exp(-x/pars[0])


def noise(x, pars):  # p0=quality: 1=uniform >10=gaussian
    return sum([random()-0.5 for _ in range(pars[0])])*sqrt(12/pars[0])


# empty class just to attach properties to


class wave:
    pass


def data_mov(start, num):
    for i in range(start, start+num):
        wavbuf[ibuf][i] = temp[i-start]


wave1 = wave()
wave1.replicate = 1
wave1.phase = 0.0

# sine
wave1.func = sine
wave1.amplitude = 0.5  # Vpp
wave1.offset = 0.0  # DC bias(V)
wave1.pars = []

# white noise
# wave1.func = noise
# wave1.amplitude = 1
# wave1.offset = 0
# wave1.pars = [1]

# sinc
# wave1.func = sinc
# wave1.amplitude = 0.5
# wave1.offset = 0
# wave1.pars = [0.01]

# pulse
# wave1.func = pulse
# wave1.amplitude = 1.0
# wave1.offset = 0.0
# # risetime(percent),uptime(percent),falltime(percent)
# # sum(risetime,uptime,falltime) should be 1
# wave1.pars = [0.1, 0.8, 0.1]

# triginal(special case of pulse)
# wave1.func = pulse
# wave1.amplitude = 1.0
# wave1.offset = 0.0
# # risetime(percent),uptime(percent),falltime(percent)
# # sum(risetime,uptime,falltime) should be 1
# wave1.pars = [0.5, 0, 0.5]

setupwave(wavbuf[ibuf], 1e5, wave1)
ibuf = (ibuf+1) % 2
