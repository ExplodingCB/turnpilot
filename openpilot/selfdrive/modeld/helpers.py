import sys
import time
from pathlib import Path

from openpilot.common.hardware import AGNOS
from openpilot.common.hardware.usb import CHESTNUT_USB_PRODUCT, get_usb_state, is_chestnut_usb_id

MODELS_DIR = Path(__file__).resolve().parent / 'models'


def modeld_pkl_path(chestnut: bool):
  prefix = 'big_' if chestnut else ''
  return MODELS_DIR / f'{prefix}driving_tinygrad.pkl'

def load_oob(path, chestnut=False):
  from tinygrad import Context
  device = 'USB+AMD:LLVM' if chestnut else 'QCOM' if AGNOS else 'METAL' if sys.platform == 'darwin' else 'CPU:LLVM'
  with Context(DEV=device):
    from tinygrad_repo.examples.openpilot.helpers import load_pickle
    return load_pickle(path, out_of_band=True)

def chestnut_present() -> bool:
  return any(is_chestnut_usb_id(d["vendorId"], d["productId"]) and d["product"] == CHESTNUT_USB_PRODUCT for d in get_usb_state())

def wait_for_chestnut(timeout: float = 10.) -> None:
  # the cable is detected before chestnut enumerates
  st = time.monotonic()
  while not chestnut_present():
    if time.monotonic() - st > timeout:
      raise TimeoutError("chestnut did not enumerate")
    time.sleep(0.1)

# the chestnut firmware's vendor request that switches the GPU's PCIe power (tinygrad's ASM24Controller.set_pcie_power)
PCIE_POWER_REQUEST = 0xF3
PCIE_OFF_S = 2.0

def reset_chestnut(off_s: float = PCIE_OFF_S) -> str:
  """TurnPilot: the GPU's PCIe power off and on through the chestnut's firmware, then a USB port reset, so the next load starts
  from a freshly powered GPU. Never raises; returns what it did, for the diagnostics.

  2026-11-06, Chase's Model 3: the big model failed to load on almost every start since 11-01, each time a different USB
  error at tinygrad's first GPU access (bulk IN timed out, bulk OUT I/O error, PCIe link not up, no such device), with the
  chestnut's 12 V steady at 14.5 V. A Tesla never cuts its lighter socket while someone is in the car, so the GPU on it is
  never power cycled between comma restarts, and a GPU left in a bad state stays in it; the one good load (11-02) came right
  after the 12 V was reconnected. The firmware switches the GPU's power itself, as a reconnect would."""
  import usb1
  devices = [d for d in get_usb_state() if is_chestnut_usb_id(d["vendorId"], d["productId"]) and d["product"] == CHESTNUT_USB_PRODUCT]
  if len(devices) != 1:
    return "reset skipped: chestnut not enumerated"
  try:
    with usb1.USBContext() as context:
      handle = context.openByVendorIDAndProductID(devices[0]["vendorId"], devices[0]["productId"], skip_on_error=True)
      if handle is None:
        return "reset skipped: chestnut couldn't be opened"
      try:
        handle.controlWrite(0x40, PCIE_POWER_REQUEST, 0, 0, b"", timeout=10000)
        time.sleep(off_s)
        handle.controlWrite(0x40, PCIE_POWER_REQUEST, 1, 0, b"", timeout=10000)
        handle.resetDevice()
      finally:
        handle.close()
    return f"GPU power cycled ({off_s:.0f} s off) and USB reset"
  except Exception as e:
    return f"reset failed: {type(e).__name__}: {e}"

def chestnut_compiled() -> bool:
  path = modeld_pkl_path(chestnut=True)
  # the big model is only built for releases, in a dev checkout it's a git LFS pointer
  return path.is_file() and path.stat().st_size > 1024 and all(
    (MODELS_DIR / f'big_driving_warp_{size}_tinygrad.pkl').is_file() for size in ('1344x760', '1928x1208'))
