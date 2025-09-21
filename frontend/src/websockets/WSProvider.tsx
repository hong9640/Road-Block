// WSProvider.tsx
import React, {
  createContext,
  useEffect,
  useMemo,
  useRef,
  useState,
} from "react";
import {
  connect as WSconnect,
  disconnect as WSdisconnect,
  sendMessage as WSsend,
} from "./WSconnection";
import { onEvent, onVehicle } from "@/websockets/handlers";
import { InfoEventModal } from "@/components/InfoEventModal";

type WSStatus = "idle" | "connecting" | "open" | "closed" | "error";
type Ctx = {
  status: WSStatus;
  send: (
    url: string,
    data: string
  ) => boolean;
};
const WSContext = createContext<Ctx | null>(null);

type Props = {
  children: React.ReactNode;
  blockUntilOpen?: boolean;
};

const mappedUrl = [
  { url: "/ws/front/vehicles", handler: onVehicle },
  { url: "/ws/front/events", handler: onEvent },
];

export function WSProvider({ children, blockUntilOpen = false }: Props) {
  const didInitRef = useRef(false);
  const [status, setStatus] = useState<WSStatus>("idle");

  // Modal 상태 관리 (EventListener의 기능 이동)
  const [open, setOpen] = useState(false);
  const [title, setTitle] = useState("알림");
  const [content, setContent] = useState<React.ReactNode>(null);

  useEffect(() => {
    if (didInitRef.current) return;
    didInitRef.current = true;

    setStatus("connecting");
    mappedUrl.forEach((item) => {
      const socket = WSconnect(item.url, (binaryData: ArrayBuffer) => {
        if (item.handler === onEvent) {
          item.handler(binaryData, {
            openModal: ({ title, content }) => {
              setTitle(title);
              setContent(content);
              setOpen(true);
            },
          });
        } else {
          item.handler(binaryData);
        }
      });
      socket.onopen = () => setStatus("open");
      socket.onclose = () => setStatus("closed");
      socket.onerror = () => setStatus("error");
    });

    return () => {
      mappedUrl.forEach((item) => WSdisconnect(item.url));
    };
  }, []);

  const api = useMemo<Ctx>(
    () => ({
      status,
      send: (url, data) => {
        try {
          WSsend(url, data);
          return true;
        } catch {
          return false;
        }
      },
    }),
    [status]
  );

  if (blockUntilOpen && status !== "open") {
    return <div style={{ padding: 16 }}>연결 중…</div>;
  }

  return (
    <WSContext.Provider value={api}>
      {children}
      {/* EventListener의 모달을 Provider 내부에 흡수 */}
      <InfoEventModal open={open} onClose={() => setOpen(false)} title={title}>
        {content}
      </InfoEventModal>
    </WSContext.Provider>
  );
}
