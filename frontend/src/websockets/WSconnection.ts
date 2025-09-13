import type { MessageHandler, SocketItem } from "@/types";

const sockets: SocketItem = {};

const WS_BASE =
  import.meta.env.VITE_WS_BASE ??
  `${location.protocol === "https:" ? "wss://" : "ws://"}${location.host}`;

export function connect(url: string, handler: MessageHandler) {
  if (!(url in sockets) || sockets[url].readyState === WebSocket.CLOSED) {
    const full = new URL(url, WS_BASE).toString();
    console.log("[WS] connecting to:", full);

    const ws = new WebSocket(full);
    sockets[url] = ws;

    ws.binaryType = "arraybuffer";
    ws.onopen = () => console.log("서버 연결 성공");
    ws.onclose = () => console.log("서버 연결 종료");
    ws.onerror = (err) => console.error("WebSocket 에러:", err);
    ws.onmessage = (event) => handler(event.data);
  }
  return sockets[url];
}

export function sendMessage(url: string, message: string) {
  const socket = sockets[url];
  if (socket?.readyState === WebSocket.OPEN) socket.send(message);
  else console.warn("WebSocket이 열려있지 않습니다.");
}

export function disconnect(url: string) {
  const ws = sockets[url];
  if (!ws) return;

  switch (ws.readyState) {
    case WebSocket.CONNECTING:
      // 아직 연결 중이면, 열리자마자 정상 종료하도록 훅만 걸고 즉시 반환
      ws.onopen = () => ws.close(1000, "cleanup");
      break;
    case WebSocket.OPEN:
    case WebSocket.CLOSING:
      ws.close(1000, "cleanup");
      break;
    case WebSocket.CLOSED:
      // no-op
      break;
  }
  // 참조 제거(핸들러에서 null 액세스 방지하려면 onclose에서 delete 해도 무방)
  delete sockets[url];
}
