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
# confident at lefts". The native full pass reads the whole 1344x760 frame, where a car 1.8 m wide is ~10 px at 200 m, about
# the smallest the detector finds: past that (40-45 mph, and roads with no mapped limit, taken as 45) lefts stay rerouted
CAR_LEFT_RANGE_M = 210.
EARLIER_CAR_RANGES_M = (150.,)  # the defaults earlier builds wrote: a comma still on one of them takes the current one


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
