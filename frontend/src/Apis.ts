import axios from "axios";
import { carsData, logsData } from "./__mocks__";

const BASE_API_URL = import.meta.env.VITE_API_BASE;

// 차량 리스트 조회
export const getVehicleListAPI = async () => {
  try {
    const response = await axios.get(`${BASE_API_URL}/vehicles`);
    console.log(response);
    return response.data.vehicles;
  } catch (e) {
    console.error(e);
    return carsData;    // 테스트용
  }
};

// 단일 차량 조회
export const getVehicleAPI = async (vehicle_id: number) => {
  try {
    const response = await axios.get(`${BASE_API_URL}/vehicles/${vehicle_id}`);
    return response.data;
  } catch (e) {
    console.error(e);
  }
};

// 사건 로그 조회
export const getEventListAPI = async () => {
  try {
    const response = await axios.get(`${BASE_API_URL}/vehicles/events`);
    return response.data.events;
  } catch (e) {
    console.error(e);
    return logsData;
  }
};

// 단일 지도 (GeoJson) 조회
export const getMapAPI = async (map_id: number) => {
  try {
    const response = await axios.get(`${BASE_API_URL}/maps/${map_id}`, {
      headers: { Accept: "application/geo+json, application/json" },
    });
    console.log(response);
    return response.data;
  } catch (e) {
    console.error(e);
  }
};
