import React, { useState, useEffect } from "react";
import ReactDOM from "react-dom";
import styles from "./Modal.module.css";

interface Patient {
  id: string;
  name: string;
  date_of_birth: string;
  sex: string;
  id_room: number;
  current_condition: string;
  professional_id: string;
}

interface ScheduleEntry {
  id: string;
  robot_id: string;
  patient_id: string;
  scheduled_time: string; // stored in UTC in the database
}

interface ModalProps {
  isOpen: boolean;
  onClose: () => void;
  robotId: string;
}

// Brasília offset: BRT = UTC-3
const BRT_OFFSET_HOURS = 3;

const Modal: React.FC<ModalProps> = ({ isOpen, onClose, robotId }) => {
  const [currentDate, setCurrentDate] = useState(new Date());
  const [patients, setPatients] = useState<Patient[]>([]);

  // Map of hour -> schedule info
  const [scheduleEntriesByHour, setScheduleEntriesByHour] = useState<Record<number, {id: string; patientId: string; scheduledTime: Date} | null>>({});

  useEffect(() => {
    const fetchPatients = async () => {
      try {
        const response = await fetch("http://localhost:8000/api/patients/", {
          headers: { "Content-Type": "application/json" },
        });
        if (!response.ok) {
          throw new Error("Failed to fetch patients");
        }
        const data = await response.json();
        setPatients(data.patients);
      } catch (err) {
        console.error("Error fetching patients:", err);
      }
    };

    fetchPatients();
  }, []);

  useEffect(() => {
    const fetchSchedulesForDay = async () => {
      try {
        const response = await fetch(`http://localhost:8000/api/robot_schedules/robot/${robotId}`, {
          headers: { "Content-Type": "application/json" },
        });
        if (!response.ok) {
          throw new Error("Failed to fetch schedules");
        }

        const data = await response.json(); // { schedules: [...] }
        const schedules: ScheduleEntry[] = data.schedules;

        // We want to compare by Brasília time. Let's convert currentDate to BRT and use that for day comparison.
        const currentDateBRT = convertToBRTTime(currentDate);
        const currentDayStrBRT = currentDateBRT.toDateString();

        const newScheduleMap: Record<number, {id: string; patientId: string; scheduledTime: Date} | null> = {};
        for (let i = 0; i < 24; i++) {
          newScheduleMap[i] = null;
        }

        schedules.forEach((schedule) => {
          // Convert schedule time from UTC to BRT
          const scheduledTimeUtc = new Date(schedule.scheduled_time);
          const scheduledTimeBrt = convertUTCToBRT(scheduledTimeUtc);
          if (scheduledTimeBrt.toDateString() === currentDayStrBRT) {
            const hour = scheduledTimeBrt.getHours();
            newScheduleMap[hour] = {
              id: schedule.id,
              patientId: schedule.patient_id,
              scheduledTime: scheduledTimeBrt,
            };
          }
        });

        setScheduleEntriesByHour(newScheduleMap);
      } catch (error) {
        console.error("Error fetching schedules:", error);
      }
    };

    if (isOpen) {
      fetchSchedulesForDay();
    }
  }, [isOpen, currentDate, robotId]);

  const handleNextDay = () => {
    setCurrentDate((prevDate) => new Date(prevDate.getTime() + 24 * 60 * 60 * 1000));
  };

  const handlePreviousDay = () => {
    setCurrentDate((prevDate) => new Date(prevDate.getTime() - 24 * 60 * 60 * 1000));
  };

  const canChangePatient = (hour: number): boolean => {
    const entry = scheduleEntriesByHour[hour];
    if (!entry) {
      return true; // No schedule yet, can create freely
    }
    const nowBRT = convertToBRTTime(new Date());
    const diffMs = entry.scheduledTime.getTime() - nowBRT.getTime();
    const diffHours = diffMs / (1000 * 60 * 60);
    // Allow changes if we are at least 2 hours away
    return diffHours > 2;
  };

  const handlePatientChange = async (hour: number, newPatientId: string) => {
    if (!canChangePatient(hour)) {
      console.warn("Cannot change schedule less than 2 hours before.");
      return;
    }

    const entry = scheduleEntriesByHour[hour];

    // If there was an existing schedule, delete it first
    if (entry && entry.id) {
      const deleteResponse = await fetch(`http://localhost:8000/api/robot_schedules/${entry.id}`, {
        method: "DELETE",
      });
      if (!deleteResponse.ok) {
        console.error("Failed to delete existing schedule before updating");
        return;
      }
    }

    // Now create the new schedule in UTC, interpreting the chosen hour as BRT.
    // currentDate is local, convert it to a BRT date (for consistency), then create a UTC ISO string from it.
    const currentDateBRT = convertToBRTTime(currentDate);
    const year = currentDateBRT.getFullYear();
    const month = currentDateBRT.getMonth();
    const day = currentDateBRT.getDate();

    // The user picks 8:00 as 8:00 BRT. To store in UTC, add BRT_OFFSET_HOURS.
    const scheduleUTCDate = new Date(Date.UTC(year, month, day, hour + BRT_OFFSET_HOURS, 0, 0, 0));

    const body = {
      robot_id: robotId,
      patient_id: newPatientId,
      scheduled_time: scheduleUTCDate.toISOString(),
    };

    try {
      const response = await fetch("http://localhost:8000/api/robot_schedules/", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });

      if (!response.ok) {
        const errorData = await response.json();
        console.error("Failed to create schedule:", errorData);
      } else {
        const createdSchedule = await response.json(); // Should return the created schedule in UTC
        console.log(`Schedule created for ${robotId}, hour ${hour}, patient ${newPatientId}`);

        // Convert the created schedule time (UTC) back to BRT for display
        const createdTimeUtc = new Date(createdSchedule.scheduled_time);
        const createdTimeBrt = convertUTCToBRT(createdTimeUtc);

        const updatedEntry = {
          id: createdSchedule.id,
          patientId: newPatientId,
          scheduledTime: createdTimeBrt,
        };

        setScheduleEntriesByHour((prev) => ({ ...prev, [hour]: updatedEntry }));
      }
    } catch (error) {
      console.error("Error creating schedule:", error);
    }
  };

  if (!isOpen) return null;

  // Format displayed date as BRT
  const currentDateBRT = convertToBRTTime(currentDate);
  const formattedDate = `${currentDateBRT.getDate().toString().padStart(2, "0")}/${
    (currentDateBRT.getMonth() + 1).toString().padStart(2, "0")
  }`;

  return ReactDOM.createPortal(
    <div className={styles.modalOverlay} onClick={onClose}>
      <div
        className={styles.modalContent}
        onClick={(e) => e.stopPropagation()} // Prevent overlay click from closing modal
      >
        <div className={styles.closeButton} onClick={onClose}>
          X
        </div>
        <div className={styles.modalHeader}>
          <h1>Calendário de Pacientes (Horário de Brasília)</h1>
          <p>Se já houver um paciente agendado, ele aparecerá pré-selecionado.  
          Alterações são permitidas somente se faltarem mais de 2 horas para o horário.</p>
        </div>
        <div className={styles.dateSelector}>
          <button className={styles.arrowButton} onClick={handlePreviousDay}>
            &lt;
          </button>
          <span className={styles.dateDisplay}>{formattedDate}</span>
          <button className={styles.arrowButton} onClick={handleNextDay}>
            &gt;
          </button>
        </div>
        <div className={styles.divider}></div>
        <div className={styles.timeSlots}>
          {Array.from({ length: 24 }, (_, index) => {
            const entry = scheduleEntriesByHour[index];
            const selectedValue = entry ? entry.patientId : "";
            const isDisabled = !canChangePatient(index);

            return (
              <div key={index} className={styles.timeSlot}>
                <span className={styles.time}>{`${index}:00`}</span>
                <select
                  className={styles.dropDown}
                  value={selectedValue}
                  disabled={isDisabled}
                  onChange={(e) => {
                    if (e.target.value) {
                      handlePatientChange(index, e.target.value);
                    }
                  }}
                >
                  <option value="" disabled>
                    Selecione um paciente
                  </option>
                  {patients.map((patient) => (
                    <option key={patient.id} value={patient.id}>
                      {patient.name}
                    </option>
                  ))}
                </select>
              </div>
            );
          })}
        </div>
      </div>
    </div>,
    document.body
  );
};

export default Modal;



function convertUTCToBRT(date: Date): Date {
  return new Date(date.getTime() - (BRT_OFFSET_HOURS * 60 * 60 * 1000));
}

function convertToBRTTime(date: Date): Date {
  return new Date(date.getTime() - (BRT_OFFSET_HOURS * 60 * 60 * 1000));
}
