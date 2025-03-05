from datetime import datetime
import pytz


def timestamp_to_influxdb_time(timestamp):
    time = datetime.utcfromtimestamp(timestamp.to_sec())
    time = pytz.timezone('UTC').localize(time).isoformat('T')
    return time
