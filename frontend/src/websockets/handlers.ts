import { useVehicleStore } from "@/stores/useVehicleStore";

// 차량 수신 정보 처리
export function onVehicle(binaryData: ArrayBuffer) {
  const view = new DataView(binaryData);
  const eventType = view.getUint8(0);

  const { updatePos } = useVehicleStore.getState();

  switch (eventType) {
    // 차량 등록
    case 0xa2: {
      const vehicle_id = view.getUint32(1, true);
      const type = view.getUint8(9);

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

      const car_name = decoder.decode(strBytes.subarray(0, end));

      console.log(vehicle_id, type, car_name);

      break;
    }

    // 차량 위치 정보 업데이트
    case 0x11: {
      const num_of_vehicle = view.getUint32(1, true);
      console.log(num_of_vehicle);

      for (let i = 0; i < num_of_vehicle; i++) {
        const vehicle_id = view.getUint32(5 + 12 * i, true);
        const posX = view.getFloat32(9 + 12 * i, true);
        const posY = view.getFloat32(13 + 12 * i, true);
        console.log(vehicle_id, posX, posY);

        updatePos(vehicle_id, posX, posY);
      }

      break;
    }

    // 차량 상태 정보 업데이트
    case 0x10: {
      const vehicle_id = view.getUint32(1, true);
      const collision_count = view.getUint8(5);
      const status_enum = view.getUint8(6);
      const status = () => {
        switch (status_enum) {
          case 0:
            return "normal";
          case 1:
            return "half_destroyed";
          case 2:
            return "complete_destroyed";
        }
      };
      const fuel = view.getUint8(7);

      console.log(vehicle_id, collision_count, status(), fuel);
      break;
    }

    // 시스템 오류
    case 0x03: {
      const errCode = view.getUint8(1);

      switch (errCode) {
        case 100:
          console.error("서버에 오류가 발생하였습니다.");
          break;
        case 101:
          console.error("DB에서 오류가 발생하였습니다.");
          break;
      }
      break;
    }

    default:
      console.warn("알 수 없는 이벤트 코드입니다.");
      break;
  }
}

// 이벤트 수신 정보 처리
export function onEvent(binaryData: ArrayBuffer) {
  const view = new DataView(binaryData);
  const eventType = view.getUint8(0);

  switch (eventType) {
    case 0:
      break;
  }
}
