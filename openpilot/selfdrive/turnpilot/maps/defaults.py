"""The map server TurnPilot ships with, so the phone only has to add the API key (the one secret, never in this repo).

The certificate is the server's public, self-signed certificate (CN turnpilot-maps, IP 136.113.85.41, until 2036),
pinned by the client like any TurnPilotMapCert. ensure_map_defaults writes the host and certificate only while their
params are empty, so a host or certificate set in the web app is kept, and server.set_map_settings's rule that a new host
or certificate needs the key in the same request still holds for changes.
"""

DEFAULT_MAP_HOST = 'https://136.113.85.41'
DEFAULT_MAP_CERT = """\
-----BEGIN CERTIFICATE-----
MIIBmTCCAT6gAwIBAgIUKZcGUg3WgPE4t/FMx7UHHdtFEOMwCgYIKoZIzj0EAwIw
GTEXMBUGA1UEAwwOdHVybnBpbG90LW1hcHMwHhcNMjYxMDI3MjM0NjQzWhcNMzYx
MDI0MjM0NjQzWjAZMRcwFQYDVQQDDA50dXJucGlsb3QtbWFwczBZMBMGByqGSM49
AgEGCCqGSM49AwEHA0IABF7K+UjKwPfYvjb2UwcwK8RFQms9AXRuifdmv+5c4Ehl
2N1Kuv6LFqWZdRzg++CiGTq6XRYQbda8nvVy6+9cUEWjZDBiMB0GA1UdDgQWBBTM
0QfS6zyx9Bhv7czXhHLi3H0CnDAfBgNVHSMEGDAWgBTM0QfS6zyx9Bhv7czXhHLi
3H0CnDAPBgNVHRMBAf8EBTADAQH/MA8GA1UdEQQIMAaHBIhxVSkwCgYIKoZIzj0E
AwIDSQAwRgIhAIVwfdMBiXkGgJCCLLb7ruJo5OtPD342pE8blhlcO0rQAiEA5a6H
D02Zf29ZEH8gUEtnp74RlS4Z59wxmPGPkP0ixo4=
-----END CERTIFICATE-----
"""
DEFAULT_MAP_CERT_SHA256 = '29:E2:71:B0:0A:27:A4:04:58:78:D4:EA:58:AF:5B:0B:F8:1B:A9:1E:D5:A9:3C:A0:99:61:41:1E:5C:4D:06:CF'


def ensure_map_defaults(params) -> list[str]:
  """Fill an empty TurnPilotMapHost and TurnPilotMapCert with the shipped server's. Returns the keys written."""
  written = []
  for key, value in (('TurnPilotMapHost', DEFAULT_MAP_HOST), ('TurnPilotMapCert', DEFAULT_MAP_CERT)):
    if not (params.get(key) or '').strip():
      params.put(key, value, block=True)
      written.append(key)
  return written


LEFT_RANGE_PARAM, LEFT_RANGE_MARK = 'TurnPilotLeftRangeM', 'TurnPilotLeftRangeProvisional'
# the car's default range for oncoming traffic: 210 m covers unprotected lefts across roads of 35 mph or less (R_req 183-203
# m at the line, 1 to 3 lanes). Kellen, 2026-11-06, after a 35 mph left was routed round at 150 m: "it just needs to be more
# confident at lefts". It is pixel arithmetic for the native schedule (TurnPilotNativeDetect), which reads the whole 1344x760
# frame at native resolution, where a car 1.8 m wide is ~10 px at 200 m, about the smallest the detector finds; never a
# measured recall. So it is the reach only while the native schedule fills the far crop's slot (turnpilotd.left_ranges:
# nativeStatus full or strip, its runs judged as the far crop's, every FAR_FRESH_S over the band past the full pass's range).
# The full pass that always runs is the frame letterboxed into 640x384, where that car is ~10 px at 98 m
# (unprotected_left.FULL_PASS_RANGE_M), and turnpilotd never certifies a band past that on its runs alone, whatever this
# says. On the car the native schedule never ran (11-06, both drives: no room beside the 38.5 ms driving model) and the far
# crop (TurnPilotLeftFarRangeM unmeasured) went off in minutes, so a left past ~98 m (every road of about 18 mph or more)
# was judged on the full pass's band alone (since 2026-11-07, no handbacks: it goes on the band it can see, a car seen there
# holding it). Past 210 m (40-45 mph, and roads with no mapped limit, taken as 45) the band reaches 210 m, the rest unseen
CAR_LEFT_RANGE_M = 210.
EARLIER_CAR_RANGES_M = (150.,)  # the defaults earlier builds wrote: a comma still on one of them takes the current one
# The far crop's provisional range (2026-11-07, bleeding edge, Kellen: "bleeding edge, not maximal caution"), what turnpilotd's
# left_ranges takes while TurnPilotLeftFarRangeM is unmeasured (0) and the far crop is on: the same pixel arithmetic as the
# 210 m above. The far crop is a 640x384 window of the road frame at native resolution (detector/graph.Crop), the native
# strips' pixel density, so a car 1.8 m wide at f 1141.5 px is 1.8 x 1141.5 / R px in it as in them: 10 px, about the
# smallest the detector boxes (FULL_PASS_RANGE_M's floor at the letterbox's 640/1344 of it), at 205.5 m, and 9.8 px at 210 m,
# Kellen's rounding, taken here too so the far slot's reach is one number whichever of the two fills it. Never a measured
# recall: a car at 210 m in the far crop is as small as one at 98 m in the full pass, whose per-frame recall was 0.27 at
# 75-100 m by day. Replaced by tools/turnpilot/left_range.py's measurement (TurnPilotLeftFarRangeM, from shadow-drive rlogs
# of oncoming cars followed through the far crop's runs, LEFT_RANGE.md): any value set wins over it. By night unmeasured
# too: the full pass boxed nothing past 78 m on 11-06's night drive (p99 50 m), the far crop's night reach was never seen.
# It only adds holds, a car going the car's own way among them: the band takes the car's own lane from 18 m out and the lane
# beside from 121 m, so a car going straight on ahead holds the left until it is past the far crop's reach (the 11-06 review,
# 2026-11-07: 15 s after the hold at a 35 mph left, against 7.8 s on the full pass's band; detector/modeld_detector's far first)
FAR_CROP_LEFT_RANGE_M = CAR_LEFT_RANGE_M


def ensure_left_range(params) -> bool:
  """An unmeasured TurnPilotLeftRangeM (0), once per comma, or a range an earlier build wrote as its default
  (EARLIER_CAR_RANGES_M), becomes CAR_LEFT_RANGE_M. Kellen, 2026-11-02: "we need the left turns, just do it and if he has
  to disengage he has to disengage". The range is pixel arithmetic, not a measured recall; any other value (a measurement,
  or one set by hand) is never touched. Returns whether it wrote the range."""
  value = params.get(LEFT_RANGE_PARAM)
  value = None if value is None else float(value)
  first = not params.get_bool(LEFT_RANGE_MARK) and (value is None or value <= 0.)
  wrote = first or value in EARLIER_CAR_RANGES_M
  if wrote:
    params.put(LEFT_RANGE_PARAM, CAR_LEFT_RANGE_M, block=True)
  params.put_bool(LEFT_RANGE_MARK, True, block=True)
  return wrote


CAMERA_LIGHTS_PARAM, CAMERA_LIGHTS_MARK = 'TurnPilotCameraLights', 'TurnPilotCameraLightsMoved'
CAMERA_LIGHTS_DEFAULT = 'addOnly'


def ensure_camera_lights(params) -> bool:
  """A comma holding TurnPilotCameraLights shadow (the default every build wrote before 2026-11-06), or nothing, takes
  addOnly, once per comma: the camera's reds stop the car (map_controls: add-only). Kellen, 2026-11-06, after the car rolled a
  right on red its reader had read in shadow: a red light is a hard rail, and no feature ships in shadow. off, or a shadow
  set after the move, is never touched. Returns whether it wrote the mode."""
  value = params.get(CAMERA_LIGHTS_PARAM)
  value = value.decode() if isinstance(value, bytes) else value
  wrote = not params.get_bool(CAMERA_LIGHTS_MARK) and value in (None, '', 'shadow')
  if wrote:
    params.put(CAMERA_LIGHTS_PARAM, CAMERA_LIGHTS_DEFAULT, block=True)
  params.put_bool(CAMERA_LIGHTS_MARK, True, block=True)
  return wrote
