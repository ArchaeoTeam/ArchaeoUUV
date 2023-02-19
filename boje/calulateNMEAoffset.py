import pyproj
import pynmea2

def apply_offset(nmea_string, offset_x, offset_y):
    if nmea_string.startswith("$GPGGA"):
        nmea_obj = pynmea2.parse(nmea_string)
    else:
        print("Not a GPGGA sentence.")
        return None

    # Get the original coordinates
    x, y = nmea_obj.longitude, nmea_obj.latitude

    # Apply the offset
    x += offset_x
    y += offset_y

    # Convert the coordinates to the new coordinate reference system
    in_crs = pyproj.CRS.from_epsg(32633)
    out_crs = pyproj.CRS.from_epsg(4326)
    transformer = pyproj.Transformer.from_crs(in_crs, out_crs)
    x, y = transformer.transform(x, y)

    # Update the NMEA object with the new coordinates
    nmea_obj.latitude = y
    nmea_obj.longitude = x  
    #Generate the new NMEA string
    new_nmea_string = pynmea2.GGA('GP', 'GGA', (nmea_obj.timestamp, 
                                                nmea_obj.latitude, 
                                                nmea_obj.lat_dir, 
                                                nmea_obj.longitude, 
                                                nmea_obj.lon_dir, 
                                                nmea_obj.gps_qual, 
                                                nmea_obj.num_sats, 
                                                nmea_obj.horizontal_dil, 
                                                nmea_obj.altitude, 
                                                nmea_obj.altitude_units, 
                                                nmea_obj.geo_sep, 
                                                nmea_obj.geo_sep_units, 
                                                nmea_obj.age_gps_data, 
                                                nmea_obj.ref_station_id))

    return new_nmea_string

# Example usage
nmea_string = "$GPGGA,081836,3751.65,S,14507.36,E,1,05,1.5,10.2,M,39.1,M,,*75"
gga = "$GPGGA,134658.00,5106.9792,N,11402.3003,W,2,09,1.0,1048.47,M"
offset_x = 100
offset_y = 50

new_nmea_string = apply_offset(nmea_string, offset_x, offset_y)
print(new_nmea_string)