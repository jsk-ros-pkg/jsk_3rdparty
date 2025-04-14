import rospy
import asyncio

from hume import HumeStreamClient
from hume.models.config import LanguageConfig

samples = [
    "Mary had a little lamb,",
    "Its fleece was white as snow.",
    "Everywhere the child went,",
    "The little lamb was sure to go."
]

async def main():
    rospy.init_node("hume_text")
    if not rospy.has_param("hume_api_key"):
        rospy.loginfo("No HUME API key")
    hume_api_key = rospy.get_param("hume_api_key")
    client = HumeStreamClient(hume_api_key)
    config = LanguageConfig()
    async with client.connect([config]) as socket:
        for sample in samples:
            result = await socket.send_text(sample)
            emotions = result["language"]["predictions"][0]["emotions"]
            print(emotions)
            print("\n")
    #rospy.spin()

asyncio.run(main())
