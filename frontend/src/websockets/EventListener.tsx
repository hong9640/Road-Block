// EventListener.tsx
import { useState } from "react";
import { InfoEventModal } from "@/components/InfoEventModal";
import { onEvent } from "@/websockets/handlers";

export default function EventListener() {
  const [open, setOpen] = useState(false);
  const [title, setTitle] = useState("알림");
  const [content, setContent] = useState<React.ReactNode>(null);

  // 이벤트 처리 함수에 state 업데이트 콜백 전달
  function handleEvent(binaryData: ArrayBuffer) {
    onEvent(binaryData, {
      openModal: ({ title, content }) => {
        setTitle(title);
        setContent(content);
        setOpen(true);
      },
    });
  }

  // 실제 WebSocket 수신 부분은 여기서 handleEvent 호출
  // ws.onmessage = (e) => handleEvent(e.data);

  return (
    <InfoEventModal open={open} onClose={() => setOpen(false)} title={title}>
      {content}
    </InfoEventModal>
  );
}
