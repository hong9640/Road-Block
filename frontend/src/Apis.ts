import axios, { AxiosError } from "axios";
const BASE_API_URL = import.meta.env.VITE_API_BASE;

// 차량 리스트 조회
export const getVehicleListAPI = async () => {
  try {
    const response = await axios.get(`${BASE_API_URL}/vehicles`);
    return response.data.vehicles;
  } catch (e) {
    axiosErrorHandler(e);
  }
};

// 단일 차량 조회
export const getVehicleAPI = async (vehicle_id: number) => {
  try {
    const response = await axios.get(`${BASE_API_URL}/vehicles/${vehicle_id}`);
    return response.data;
  } catch (e) {
    axiosErrorHandler(e);
  }
};

// 단일 차량 조회
export const deleteVehicleAPI = async (vehicle_id: number) => {
  try {
    const response = await axios.delete(
      `${BASE_API_URL}/vehicles/${vehicle_id}`
    );
    return response;
  } catch (e) {
    axiosErrorHandler(e);
  }
};

// 사건 로그 조회
export const getEventListAPI = async () => {
  try {
    const response = await axios.get(`${BASE_API_URL}/vehicles/events`);
    return response.data.events;
  } catch (e) {
    axiosErrorHandler(e);
  }
};

// 단일 지도 (GeoJson) 조회
export const getMapAPI = async (map_id: number) => {
  try {
    const response = await axios.get(`${BASE_API_URL}/maps/${map_id}`, {
      headers: { Accept: "application/geo+json, application/json" },
    });
    return response.data;
  } catch (e) {
    axiosErrorHandler(e);
  }
};

function axiosErrorHandler(err: unknown) {
  if (axios.isAxiosError(err)) {
    const axiosError = err as AxiosError;
    switch (axiosError.message) {
      case "Network Error":
        console.error("로딩 실패: 네트워크가 연결되어 있지 않습니다.");
        break;
    }
  } else {
    console.log("Unknown Error:", err);
  }
}
