// ws/WebSocketProvider.tsx
import React, { createContext, useEffect, useMemo, useRef, useState } from "react";
import {
  connect as WSconnect,
  disconnect as WSdisconnect,
  sendMessage as WSsend,
} from "./WSconnection";
import { onEvent, onVehicle } from "@/websockets/handlers";

type WSStatus = "idle" | "connecting" | "open" | "closed" | "error";
type Ctx = {
  status: WSStatus;
  send: (
    url: string,
    data: string | ArrayBufferLike | Blob | ArrayBufferView
  ) => boolean;
};
const WSContext = createContext<Ctx | null>(null);

type Props = {
  children: React.ReactNode;
  blockUntilOpen?: boolean;
};

const mappedUrl = [
  { url: "/ws/vehicles", handler: onVehicle },
  { url: "/ws/events", handler: onEvent },
];

export function WSProvider({ children, blockUntilOpen = false }: Props) {
  const didInitRef = useRef(false);
  const [status, setStatus] = useState<WSStatus>("idle");

  useEffect(() => {
    if (didInitRef.current) return;
    didInitRef.current = true;

    setStatus("connecting");
    mappedUrl.forEach((item) => {
      const socket = WSconnect(item.url, item.handler);
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

  return <WSContext.Provider value={api}>{children}</WSContext.Provider>;
}
