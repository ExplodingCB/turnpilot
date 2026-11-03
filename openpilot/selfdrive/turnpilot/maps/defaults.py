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


def ensure_left_range(params) -> bool:
  """Once per comma: an unmeasured TurnPilotLeftRangeM (0) becomes unprotected_left.PROVISIONAL_RANGE_M, so the car drives
  unprotected lefts across roads of 25 mph or less (R_req 134-141 m) instead of routing around or skipping every one.
  Kellen, 2026-11-02: "we need the left turns, just do it and if he has to disengage he has to disengage". The range is the
  pixel arithmetic's (a car ~6.5 px wide at 150 m), not a measured recall; a measured value set later is never touched.
  Returns whether it wrote the range."""
  from openpilot.selfdrive.turnpilot.unprotected_left import PROVISIONAL_RANGE_M
  if params.get_bool(LEFT_RANGE_MARK):
    return False
  value = params.get(LEFT_RANGE_PARAM)
  wrote = value is None or float(value) <= 0.
  if wrote:
    params.put(LEFT_RANGE_PARAM, PROVISIONAL_RANGE_M, block=True)
  params.put_bool(LEFT_RANGE_MARK, True, block=True)
  return wrote
