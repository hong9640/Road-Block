export function onVehicle(binaryData: ArrayBuffer) {
  const view = new DataView(binaryData);
  const eventType = view.getUint8(0);

  console.log(binaryData);
  console.log(eventType);

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
      const vehicle_id = view.getUint32(1, true);
      const posX = view.getFloat32(5, true);
      const posY = view.getFloat32(9, true);

      console.log(vehicle_id, posX, posY);

      break;
    }

    // 차량 상태 정보 업데이트
    case 0x10: {
      const vehicle_id = view.getUint32(1, true);
      const collision_count = view.getUint8(5);
      const status_enum = view.getUint8(6);
      const fuel = view.getUint8(7);

      console.log(vehicle_id, collision_count, status_enum, fuel);
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

export function onEvent() {}
