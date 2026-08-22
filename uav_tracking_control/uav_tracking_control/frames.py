import math

def enu_to_ned(x, y, z):
    return y, x, -z

def ned_to_enu(north, east, down):
    return east, north, -down

def local_ned_to_global(north, east, down, ref_lat, ref_lon, ref_alt):
    EARTH_RADIUS = 6378137.0
    dlat = north / EARTH_RADIUS
    dlon = east / (EARTH_RADIUS * math.cos(ref_lat * math.pi / 180.0))
    lat = ref_lat + dlat * 180.0 / math.pi
    lon = ref_lon + dlon * 180.0 / math.pi
    alt = ref_alt - down
    return lat, lon, alt
