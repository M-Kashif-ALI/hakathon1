import React from 'react';
import clsx from 'clsx';
import styles from './SensorDataViz.module.css';

// Define the props for the SensorDataViz component
type SensorDataVizProps = {
  title: string;
  description: string;
  sensorType: 'lidar' | 'camera' | 'imu' | 'other';
  dataPreview?: string;
  className?: string;
};

const SensorDataViz: React.FC<SensorDataVizProps> = ({
  title,
  description,
  sensorType,
  dataPreview,
  className,
}) => {
  // Get icon based on sensor type
  const getSensorIcon = () => {
    switch (sensorType) {
      case 'lidar':
        return '📡';
      case 'camera':
        return '📷';
      case 'imu':
        return '⚙️';
      default:
        return '🔍';
    }
  };

  return (
    <div className={clsx('card', styles.sensorDataViz, className)}>
      <div className="card__header">
        <h3>{getSensorIcon()} {title}</h3>
      </div>
      <div className="card__body">
        <p>{description}</p>
        {dataPreview && (
          <div className={styles.dataPreview}>
            <h4>Data Preview:</h4>
            <pre className={styles.dataContent}>{dataPreview}</pre>
          </div>
        )}
      </div>
    </div>
  );
};

export default SensorDataViz;