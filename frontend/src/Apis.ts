import axios from "axios";
const BASE_API_URL = import.meta.env.VITE_API_BASE;

// 차량 리스트 조회
export const getVehicleListAPI = async () => {
  console.log("API 요청 직전 BASE_API_URL 값:", BASE_API_URL);
  try {
    const response = await axios.get(`${BASE_API_URL}/vehicles`);
    return response.data.vehicles;
  } catch (e) {
    console.error(e);
  }
};

// 단일 차량 조회
export const getVehicleAPI = async (vehicle_id: number) => {
  console.log("API 요청 직전 BASE_API_URL 값:", BASE_API_URL);
  try {
    const response = await axios.get(`${BASE_API_URL}/vehicles/${vehicle_id}`);
    return response.data;
  } catch (e) {
    console.error(e);
  }
};

// 단일 차량 조회
export const deleteVehicleAPI = async (vehicle_id: number) => {
  console.log("API 요청 직전 BASE_API_URL 값:", BASE_API_URL);
  try {
    const response = await axios.delete(`${BASE_API_URL}/vehicles/${vehicle_id}`);
    return response;
  } catch (e) {
    console.error(e);
  }
};

// 사건 로그 조회
export const getEventListAPI = async () => {
  console.log("API 요청 직전 BASE_API_URL 값:", BASE_API_URL);
  try {
    const response = await axios.get(`${BASE_API_URL}/vehicles/events`);
    return response.data.events;
  } catch (e) {
    console.error(e);
  }
};

// 단일 지도 (GeoJson) 조회
export const getMapAPI = async (map_id: number) => {
  console.log("API 요청 직전 BASE_API_URL 값:", BASE_API_URL);
  try {
    const response = await axios.get(`${BASE_API_URL}/maps/${map_id}`, {
      headers: { Accept: "application/geo+json, application/json" },
    });
    return response.data;
  } catch (e) {
    console.error(e);
  }
};
