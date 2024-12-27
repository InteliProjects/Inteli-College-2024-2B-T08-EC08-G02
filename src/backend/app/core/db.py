from motor.motor_asyncio import AsyncIOMotorClient, AsyncIOMotorCollection

# TODO: Change the localhost to the container name in the future (mongodb)
MONGO_URI = "mongodb://root:password@mongodb:27017"
DB_NAME = "cora"


class MongoDB:
    def __init__(self, uri: str, db_name: str):
        self.client = AsyncIOMotorClient(uri)
        self.db = self.client[db_name]

    def get_collection(self, collection_name: str) -> AsyncIOMotorCollection:
        return self.db[collection_name]

    async def close(self):
        self.client.close()


# Instância global de MongoDB
mongo_db = MongoDB(MONGO_URI, DB_NAME)
