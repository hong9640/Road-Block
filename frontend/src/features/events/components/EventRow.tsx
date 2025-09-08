import clsx from "clsx";

type Status = "RUN" | "CATCH";
export type Log = {
  id: number;
  catcher_id: number;
  runner_id: number;
  status: Status;
  created_at: string;      // ISO 8601
};

interface EventRowProps { log: Log; }

function handleStatus(
  event: Status,
  catcher?: number,
  runner?: number
) {
  switch (event) {
    case "RUN":
      return {
        style: "bg-yellow-100",
        eventWord: "추적 시작",
        detail: `${catcher}번 경찰차가 ${runner}번 도주 차량 추격을 시작했습니다.`,
      };
    case "CATCH":
      return {
        style: "bg-green-100",
        eventWord: "검거 완료",
        detail: `${catcher}번 경찰차가 ${runner}번 도주 차량을 검거했습니다.`,
      };
  }
}

function formatDate(iso: string) {
  // 2025-09-08
  const d = new Date(iso);
  const y = d.getFullYear();
  const m = String(d.getMonth() + 1).padStart(2, "0");
  const day = String(d.getDate()).padStart(2, "0");
  return `${y}-${m}-${day}`;
}
function formatTime(iso: string) {
  // 10:24:49 (초까지)
  const d = new Date(iso);
  const hh = String(d.getHours()).padStart(2, "0");
  const mm = String(d.getMinutes()).padStart(2, "0");
  const ss = String(d.getSeconds()).padStart(2, "0");
  return `${hh}:${mm}:${ss}`;
}

export default function EventRow({ log }: EventRowProps) {
  const st = handleStatus(log.status, log.catcher_id, log.runner_id);

  return (
    <tr className="odd:bg-white even:bg-slate-50">
      <td className={clsx(st.style, "px-3 py-2 whitespace-nowrap text-center")}>
        <time dateTime={log.created_at}>{formatDate(log.created_at)}</time>
      </td>
      <td className={clsx(st.style, "px-3 py-2 whitespace-nowrap text-center")}>
        <time dateTime={log.created_at}>{formatTime(log.created_at)}</time>
      </td>
      <td className={clsx(st.style, "px-3 py-2 text-center font-medium")}>
        {st.eventWord}
      </td>
      <td className={clsx(st.style, "px-3 py-2")}>
        <span className="block truncate sm:whitespace-normal">{st.detail}</span>
      </td>
    </tr>
  );
}
