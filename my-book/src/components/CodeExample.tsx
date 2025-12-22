import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

type Props = {
  children: React.ReactNode;
  language?: string;
  title?: string;
  description?: string;
};

const CodeExample: React.FC<Props> = ({ children, language = 'python', title, description }) => {
  return (
    <div className="code-example-container">
      {title && <h4 className="code-example-title">{title}</h4>}
      {description && <p className="code-example-description">{description}</p>}
      <div className="code-example-content">
        <pre className={`language-${language}`}>
          <code>{children}</code>
        </pre>
      </div>
    </div>
  );
};

export default CodeExample;