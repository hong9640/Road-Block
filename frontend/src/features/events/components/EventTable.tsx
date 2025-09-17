import EventRow from "./EventRow";
import { useEventStore } from "@/stores/useEventStore";

export default function EventTable() {
  const events = useEventStore((s) => s.events);

  return (
    <main>
      <h1 className="mb-6 text-3xl">사건 기록</h1>

      {/* 데스크톱/태블릿: 테이블 */}
      <div className="hidden md:block border border-slate-200">
        <div className="max-h-[70vh] overflow-auto">
          <table className="min-w-full table-fixed">
            {/* 고정 폭 지정 */}
            <colgroup>
              <col className="w-28" />
              <col className="w-28" />
              <col className="w-28" />
              <col />
            </colgroup>

            <thead className="sticky top-0 z-10 bg-white shadow-[0_1px_0_0_rgba(0,0,0,0.06)]">
              <tr>
                <th
                  scope="col"
                  className="px-3 py-2 text-center text-slate-600"
                >
                  날짜
                </th>
                <th
                  scope="col"
                  className="px-3 py-2 text-center text-slate-600"
                >
                  시간
                </th>
                <th
                  scope="col"
                  className="px-3 py-2 text-center text-slate-600"
                >
                  분류
                </th>
                <th
                  scope="col"
                  className="px-3 py-2 text-center text-slate-600"
                >
                  상세
                </th>
              </tr>
            </thead>
            <tbody>
              {events.map((log) => (
                <EventRow key={log.id} log={log} />
              ))}
            </tbody>
          </table>
        </div>
      </div>
    </main>
  );
}
