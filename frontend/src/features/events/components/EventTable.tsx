import { useState, useMemo } from "react";
import EventRow from "./EventRow";
import { useEventStore } from "@/stores/useEventStore";
import "./EventTable.css";

export default function EventTable() {
  const events = useEventStore((s) => s.events);

  // 👉 필터 상태
  const [dateFilter, setDateFilter] = useState<string>(""); // yyyy-mm-dd
  const [typeFilter, setTypeFilter] = useState<string>("all"); // 사건 유형

  // 👉 필터링된 이벤트
  const filteredEvents = useMemo(() => {
    if (!events) return [];

    return events.filter((log) => {
      // 날짜 필터 (created_at → yyyy-mm-dd 추출)
      const logDate = log.created_at.slice(0, 10); // "YYYY-MM-DD"
      const matchDate = !dateFilter || logDate === dateFilter;

      // 사건 유형 필터 (RunnerStatus 기준)
      const matchType = typeFilter === "all" || log.status === typeFilter;

      return matchDate && matchType;
    });
  }, [events, dateFilter, typeFilter]);

  // 👉 상태 분기 처리
  if (!events) {
    return (
      <main>
        <h1 className="mb-8 text-4xl">사건 기록</h1>
        <div className="p-6 info-text">
          사건 기록을 불러오는 데 실패하였습니다.
        </div>
      </main>
    );
  }

  if (events.length === 0) {
    return (
      <main>
        <h1 className="mb-8 text-4xl">사건 기록</h1>
        <div className="p-6 info-text">데이터를 불러오는 중입니다...</div>
      </main>
    );
  }

  return (
    <main>
      <div className="flex justify-between mb-8">
        <h1 className="text-4xl">사건 기록</h1>

        {/* 필터 UI */}
        <div className="flex gap-4 mr-4">
          {/* 날짜 필터 */}
          <input
            type="date"
            value={dateFilter}
            onChange={(e) => setDateFilter(e.target.value)}
            className="event-table-filter"
          />

          {/* 사건 유형 필터 */}
          <select
            value={typeFilter}
            onChange={(e) => setTypeFilter(e.target.value)}
            className="event-table-filter"
          >
            <option value="all">전체</option>
            <option value="escaped">도주</option>
            <option value="caught">검거</option>
            <option value="pending">대기</option>
            {/* RunnerStatus enum에 맞게 추가 */}
          </select>
        </div>
      </div>

      {/* 데스크톱/태블릿 전용 */}
      <div className="event-table-container">
        <div className="event-table-scroll">
          <table className="event-table">
            <colgroup>
              <col className="event-table-col-date" />
              <col className="event-table-col-time" />
              <col className="event-table-col-type" />
              <col />
            </colgroup>

            <thead className="event-table-head">
              <tr>
                <th scope="col" className="event-table-head-cell">
                  날짜
                </th>
                <th scope="col" className="event-table-head-cell">
                  시간
                </th>
                <th scope="col" className="event-table-head-cell">
                  분류
                </th>
                <th scope="col" className="event-table-head-cell">
                  상세
                </th>
              </tr>
            </thead>

            <tbody>
              {filteredEvents.length === 0 ? (
                <tr>
                  <td colSpan={4} className="py-6 info-text">
                    조건에 맞는 사건 기록이 없습니다.
                  </td>
                </tr>
              ) : (
                filteredEvents.map((log) => <EventRow key={log.id} log={log} />)
              )}
            </tbody>
          </table>
        </div>
      </div>
    </main>
  );
}
