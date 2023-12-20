
#!/bin/sh


rm lynx_driver/include/lynx_driver/mavlink -rf
mavgen.py -o lynx_driver/include/lynx_driver/mavlink --lang C --wire-protocol 1.0 lynx_firmware/xml/lynx.xml

rm lynx_firmware/include/lynx_firmware/mavlink -rf
mavgen.py -o lynx_firmware/include/lynx_firmware/mavlink --lang C --wire-protocol 1.0 lynx_firmware/xml/lynx.xml


sed -i 's/#define MAVLINK_MAX_FIELDS 64/#define MAVLINK_MAX_FIELDS 16/g' lynx_firmware/include/lynx_firmware/mavlink/mavlink_types.h
sed -i 's/#define MAVLINK_MAX_EXTENDED_PACKET_LEN 65507/#define MAVLINK_MAX_EXTENDED_PACKET_LEN 1024/g' lynx_firmware/include/lynx_firmware/mavlink/mavlink_types.h
sed -i 's/#define MAVLINK_MAX_PAYLOAD_LEN 255/#define MAVLINK_MAX_PAYLOAD_LEN 32/g' lynx_firmware/include/lynx_firmware/mavlink/mavlink_types.h
sed -i 's/# define MAVLINK_COMM_NUM_BUFFERS 4/# define MAVLINK_COMM_NUM_BUFFERS 1/g' lynx_firmware/include/lynx_firmware/mavlink/mavlink_types.h
