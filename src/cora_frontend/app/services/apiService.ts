import axios from "axios";
import { getUserIdFromToken } from "@/components/utils/auth";

const API_BASE_URL = "http://localhost:8000/api/"; // Substitua pela URL do backend

const api = axios.create({
    baseURL: API_BASE_URL,
    headers: {
        "Content-Type": "application/json",
    },
});

// Patient related endpoints

export const getAllPatients = async () => {
    const response = await api.get("/patients/");
    return response.data;
}

export const getPatients = async () => {
    const response = await api.get("/patients/professional/" + getUserIdFromToken());
    return response.data;
};

export const createPatient = async (patientData: any) => {
    const response = await api.post("/patients/", patientData);
    return response.data;
};

export const searchPatientByName = async (name: string) => {
    const response = await api.get(`/patients/search/`, { params: { name } });
    return response.data;
};

export const attrelatePatient = async (patientID: string) => {
    const response = await api.put(`/patients/${patientID}/associate/` + getUserIdFromToken());
    return response.data;
}

export const deattrelatePatient = async (patientID: string) => {
    const response = await api.put(`/patients/${patientID}/disassociate`);
    return response.data;
}

// Questions/Answers related endpoints



export default api;
