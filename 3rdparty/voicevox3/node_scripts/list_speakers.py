from voicevox import Client
import asyncio


async def main():
    async with Client() as client:
        # check core
        for version in await client.fetch_core_versions():
            print("Core version: {}".format(version))
        # check engine
        engine_version = await client.fetch_engine_version()
        print("Engine version: {}".format(engine_version))
        # check device
        for device in await client.http.request("GET", "/supported_devices"):
            print("Device: {}".format(device))
        # check speaker
        for speaker in await client.fetch_speakers():
            print(speaker.uuid, speaker.name, speaker.supported_features.permitted_synthesis_morphing)
            for styles in speaker.styles:
                print(styles.id, speaker.name, styles.name)

if __name__ == "__main__":
    asyncio.run(main())

    
