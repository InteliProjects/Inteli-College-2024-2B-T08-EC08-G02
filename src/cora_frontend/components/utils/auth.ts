import {jwtDecode} from "jwt-decode";

export const getUserIdFromToken = (): string | null => {
    if (typeof window === "undefined") return null; 
    const token = localStorage.getItem("token");
    if (!token) return null;

    try {
        const decodedToken: any = jwtDecode(token);
        console.log(decodedToken);
        return decodedToken.id; 
    } catch (error) {
        console.error("Invalid token:", error);
        return null;
    }
};

export const getUserRoleFromToken = (): string | null => {
    if (typeof window === "undefined") return null; 
    const token = localStorage.getItem("token");
    if (!token) return null;

    try {
        const decodedToken: any = jwtDecode(token);
        return decodedToken.role; 
    } catch (error) {
        console.error("Invalid token:", error);
        return null;
    }
}