import React from 'react';
import clsx from 'clsx';
import styles from './GazeboDiagram.module.css';

// Define the props for the GazeboDiagram component
type GazeboDiagramProps = {
  title: string;
  description: string;
  imageUrl?: string;
  className?: string;
};

// Default image URL for Gazebo simulation diagrams
const DEFAULT_GAZEBO_IMAGE = '/img/gazebo-simulation.png';

const GazeboDiagram: React.FC<GazeboDiagramProps> = ({
  title,
  description,
  imageUrl = DEFAULT_GAZEBO_IMAGE,
  className,
}) => {
  return (
    <div className={clsx('card', styles.gazeboDiagram, className)}>
      <div className="card__header">
        <h3>{title}</h3>
      </div>
      <div className="card__body">
        <img
          src={imageUrl}
          alt={title}
          className={styles.diagramImage}
        />
        <p>{description}</p>
      </div>
    </div>
  );
};

export default GazeboDiagram;