import type { Log, RunnerStatus } from "@/types";
import { formatDateYYYYMMDD, formatTimeHHMMSS } from "@/utils/timeformat";
import clsx from "clsx";
import "./EventRow.css"

interface EventRowProps {
  log: Log;
}

function handleStatus(
  event: RunnerStatus,
  catcher?: number | null,
  runner?: number
) {
  switch (event) {
    case "run":
      return {
        style: "event-status-run",
        eventWord: "추적 시작",
        detail: `${runner}번 도주 차량 추격을 시작했습니다.`,
      };
    case "catch":
      return {
        style: "event-status-catch",
        eventWord: "검거 완료",
        detail: `${catcher}번 경찰차가 ${runner}번 도주 차량을 검거했습니다.`,
      };
  }
}

export default function EventRow({ log }: EventRowProps) {
  const st = handleStatus(log.status, log.catcher_id, log.runner_id);

  return (
    <tr className="event-row">
      <td className={clsx(st.style, "event-cell")}>
        <time dateTime={log.created_at}>
          {formatDateYYYYMMDD(log.created_at)}
        </time>
      </td>
      <td className={clsx(st.style, "event-cell")}>
        <time dateTime={log.created_at}>
          {formatTimeHHMMSS(log.created_at)}
        </time>
      </td>
      <td className={clsx(st.style, "event-cell font-medium")}>
        {st.eventWord}
      </td>
      <td className={clsx(st.style, "event-cell-detail")}>
        <span className="block truncate sm:whitespace-normal">{st.detail}</span>
      </td>
    </tr>
  );
}
