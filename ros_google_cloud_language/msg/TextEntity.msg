# ROS interface for https://googleapis.dev/python/language/latest/language_v1/types.html#google.cloud.language_v1.types.Entity

string name
int32 type

# Enumerate of entity type
# https://googleapis.dev/python/language/latest/language_v1/types.html#google.cloud.language_v1.types.Entity.Type
uint8 ADDRESS = 10
uint8 CONSUMER_GOOD = 6
uint8 DATE = 11
uint8 EVENT = 4
uint8 LOCATION = 2
uint8 NUMBER = 12
uint8 ORGANIZATION = 3
uint8 OTHER = 7
uint8 PERSON = 1
uint8 PHONE_NUMBER = 9
uint8 PRICE = 13
uint8 UNKNOWN = 0
uint8 WORK_OF_ART = 5

diagnostic_msgs/KeyValue[] metadata
float32 salience
