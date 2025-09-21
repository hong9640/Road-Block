import { useEventStore } from "@/stores/useEventStore";
import { useVehicleStore } from "@/stores/useVehicleStore";
import { type DamageLevel, type VehicleType } from "@/types";

const VEHICLE_TYPE_MAP: Record<number, VehicleType> = {
  0: "police",
  1: "runner",
} as const;

const DAMAGE_LEVEL_MAP: Record<number, DamageLevel> = {
  0: "normal",
  1: "half_destroyed",
  2: "complete_destroyed",
} as const;

const { addCar, updatePos, updateStatus, deleteCar } =
  useVehicleStore.getState();
const { addEvents } = useEventStore.getState();

// 차량 수신 정보 처리
export function onVehicle(binaryData: ArrayBuffer) {
  const view = new DataView(binaryData);
  const eventType = view.getUint8(0);

  switch (eventType) {
    // 차량 등록
    case 0xa2: {
      const id = view.getUint32(1, true);
      const type_num = view.getUint8(9);

      const decoder = new TextDecoder("utf-8"); // 인코딩 명시
      const strBytes = new Uint8Array(binaryData, 10, 10); // offset 8, 길이 20

      // c style null 문자 처리
      let end = strBytes.length;
      for (let i = 0; i < strBytes.length; i++) {
        if (strBytes[i] === 0x00) {
          end = i;
          break;
        }
      }

      const vehicle_type = VEHICLE_TYPE_MAP[type_num];
      const car_name = decoder.decode(strBytes.subarray(0, end));

      addCar({ id, car_name, vehicle_type, details: null });
      break;
    }

    // 차량 위치 정보 업데이트
    case 0x11: {
      const num_of_vehicle = view.getUint32(1, true);

      for (let i = 0; i < num_of_vehicle; i++) {
        const vehicle_id = view.getUint32(5 + 12 * i, true);
        const posX = view.getFloat32(9 + 12 * i, true);
        const posY = view.getFloat32(13 + 12 * i, true);

        updatePos(vehicle_id, posX, posY);
      }

      break;
    }

    // 차량 상태 정보 업데이트
    case 0x10: {
      const vehicle_id = view.getUint32(1, true);
      const colision_count = view.getUint8(5);
      const status_enum = view.getUint8(6);
      const fuel = view.getUint8(7);

      const status = DAMAGE_LEVEL_MAP[status_enum];

      updateStatus(vehicle_id, { colision_count, status, fuel });

      break;
    }

    // 시스템 오류
    case 0x03: {
      const errCode = view.getUint8(1);
      errorLog(errCode);
      break;
    }

    default:
      break;
  }
}

// 이벤트 수신 정보 처리
export function onEvent(
  binaryData: ArrayBuffer,
  opts?: {
    openModal: (p: { title: string; content: React.ReactNode }) => void;
  }
) {
  const view = new DataView(binaryData);
  const eventType = view.getUint8(0);

  switch (eventType) {
    // 추적 시작
    case 0xf0: {
      const runner_id = view.getUint32(1, true);
      console.log(runner_id);
      addEvents(runner_id, null, "run");
      break;
    }

    // 추적 실패 (시연에는 없음)
    case 0xfd: {
      break;
    }

    // 검거 성공
    case 0xfe: {
      const catcher_id = view.getUint32(1, true);
      const runner_id = view.getUint32(5, true);
      console.log(catcher_id, runner_id);
      addEvents(runner_id, catcher_id, "catch");
      deleteCar(runner_id);
      break;
    }

    // 에러 발생
    case 0x03: {
      const errCode = view.getUint8(1);
      errorLog(errCode);
      break;
    }

    default:
      console.warn("알 수 없는 이벤트 코드입니다.");
      break;
  }
}

function errorLog(errCode: number) {
  switch (errCode) {
    case 100:
      console.error("서버에 오류가 발생하였습니다.");
      break;
    case 101:
      console.error("DB 저장에 실패하였습니다.");
      break;
  }
}

export function onTest(binaryData: ArrayBuffer) {
  console.log(binaryData);
}
