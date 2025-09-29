import { useState } from "react";
import { InfoEventModal } from "./InfoEventModal";

export default function TestModal() {
  const [open, setOpen] = useState(false);

  return (
    <div className="p-4">
      <button
        type="button"
        className="px-4 py-2 bg-blue-500 text-white rounded"
        onClick={() => setOpen(true)}
      >
        모달 열기
      </button>

      <InfoEventModal
        open={open}
        onClose={() => setOpen(false)}
        title="테스트 모달"
        autoCloseMs={5000}
      >
        <p>이것은 모달 테스트입니다.</p>
        <p>
          버튼을 누르면 모달이 열리고, ESC 키나 배경 클릭, 닫기 버튼으로 닫을 수
          있습니다.
        </p>
      </InfoEventModal>
    </div>
  );
}
