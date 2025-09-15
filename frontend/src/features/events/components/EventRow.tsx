import type { Log, Status } from "@/types";
import { formatDateYYYYMMDD, formatTimeHHMMSS } from "@/utils/timeformat";
import clsx from "clsx";

interface EventRowProps {
  log: Log;
}

function handleStatus(event: Status, catcher?: number, runner?: number) {
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

export default function EventRow({ log }: EventRowProps) {
  const st = handleStatus(log.status, log.catcher_id, log.runner_id);

  return (
    <tr className="odd:bg-white even:bg-slate-50">
      <td className={clsx(st.style, "px-3 py-2 whitespace-nowrap text-center")}>
        <time dateTime={log.created_at}>
          {formatDateYYYYMMDD(log.created_at)}
        </time>
      </td>
      <td className={clsx(st.style, "px-3 py-2 whitespace-nowrap text-center")}>
        <time dateTime={log.created_at}>
          {formatTimeHHMMSS(log.created_at)}
        </time>
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
