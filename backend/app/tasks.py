# app/tasks.py (신규 파일)

from app.celery_app import celery_app
from app.db_sync import SessionMaker 
from app.models.models import VehicleLocation

@celery_app.task
def save_location_task(vehicle_id: int, pos_x: float, pos_y: float):
    """Celery 워커가 비동기적으로 위치 정보를 DB에 저장하는 작업"""
    print(f"Celery 워커: 차량 {vehicle_id} 위치 저장 시작.")
    
    with SessionMaker() as db_session:
        try:
            new_location = VehicleLocation(
                vehicle_id=vehicle_id,
                position_x=pos_x,
                position_y=pos_y
            )
            db_session.add(new_location)
            db_session.commit()
            print(f"Celery 워커: 차량 {vehicle_id} 위치 저장 완료.")
        except Exception as e:
            db_session.rollback()
            print(f"Celery 워커: DB 저장 에러 - {e}")