import React from 'react';
import clsx from 'clsx';
import styles from './UnityViewer.module.css';

// Define the props for the UnityViewer component
type UnityViewerProps = {
  title: string;
  description: string;
  imageUrl?: string;
  className?: string;
};

// Default image URL for Unity environment diagrams
const DEFAULT_UNITY_IMAGE = '/img/unity-environment.png';

const UnityViewer: React.FC<UnityViewerProps> = ({
  title,
  description,
  imageUrl = DEFAULT_UNITY_IMAGE,
  className,
}) => {
  return (
    <div className={clsx('card', styles.unityViewer, className)}>
      <div className="card__header">
        <h3>{title}</h3>
      </div>
      <div className="card__body">
        <img
          src={imageUrl}
          alt={title}
          className={styles.viewerImage}
        />
        <p>{description}</p>
      </div>
    </div>
  );
};

export default UnityViewer;