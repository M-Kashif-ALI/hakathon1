import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

type Props = {
  children: React.ReactNode;
  title?: string;
  description?: string;
  type?: 'architecture' | 'flow' | 'component' | 'sequence';
};

const ROSDiagram: React.FC<Props> = ({ children, title, description, type = 'architecture' }) => {
  const diagramClass = `ros-diagram ros-diagram-${type}`;

  return (
    <div className={diagramClass}>
      {title && <h4 className="diagram-title">{title}</h4>}
      {description && <p className="diagram-description">{description}</p>}
      <div className="diagram-content">
        {children}
      </div>
    </div>
  );
};

export default ROSDiagram;