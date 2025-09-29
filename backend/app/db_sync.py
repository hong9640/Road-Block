# app/db_sync.py

import os
import json
from dotenv import load_dotenv
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

load_dotenv()

DB_HOST = os.getenv("DB_HOST")
DB_PORT = os.getenv("DB_PORT")
DB_USERNAME = os.getenv("DB_USERNAME")
DB_PASSWORD = os.getenv("DB_PASSWORD")
DB_NAME = os.getenv("DB_NAME")
DB_SSL_CONFIG = os.getenv("DB_SSL_CONFIG")

if not all([DB_HOST, DB_PORT, DB_USERNAME, DB_PASSWORD, DB_NAME]):
    raise ValueError("App Error: Missing database configuration in .env file.")

DATABASE_URL = f"mysql+pymysql://{DB_USERNAME}:{DB_PASSWORD}@{DB_HOST}:{DB_PORT}/{DB_NAME}"

connect_args = {}
if DB_SSL_CONFIG:
    try:
        ssl_config = json.loads(DB_SSL_CONFIG)
        connect_args["ssl"] = ssl_config
    except json.JSONDecodeError:
        if DB_SSL_CONFIG.lower() == "true":
            connect_args["ssl"] = True
        else:
            raise ValueError(f"Invalid DB_SSL_CONFIG format: {DB_SSL_CONFIG}")

engine = create_engine(
    DATABASE_URL,
    connect_args=connect_args
)

SessionMaker = sessionmaker(autocommit=False, autoflush=False, bind=engine)