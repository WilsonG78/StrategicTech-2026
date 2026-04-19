std_msgs/Header header

# QR code raw content (used as box ID)
string qr_data

# Object classification placeholder – will be replaced by ML model later
string object_type

# Object GPS position
float64 latitude
float64 longitude
float64 altitude_m

# Annotated photo: bbox, object_type, coordinates, box ID overlaid
sensor_msgs/CompressedImage photo

# Top-down 2D map: lawnmower path + detected object markers
sensor_msgs/CompressedImage map_image
