import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import {useLocation} from '@docusaurus/router';
import {useBlogPost} from '@docusaurus/theme-common/internal';
import {translate} from '@docusaurus/Translate';
import styles from './Breadcrumb.module.css';

function useBreadcrumbs() {
  const location = useLocation();
  const {metadata: blogPostMetadata} = useBlogPost() || {};

  // For blog posts
  if (blogPostMetadata) {
    return [
      {label: translate({message: 'Blog'}), href: '/blog'},
      {label: blogPostMetadata.title, href: location.pathname},
    ];
  }

  // For docs
  const pathSegments = location.pathname
    .split('/')
    .filter(segment => segment.length > 0 && segment !== 'docs');

  if (pathSegments.length > 0) {
    const breadcrumbs = [{label: translate({message: 'Docs'}), href: '/docs'}];

    let currentPath = '/docs';
    pathSegments.forEach((segment, index) => {
      currentPath += `/${segment}`;
      if (index < pathSegments.length - 1) {
        breadcrumbs.push({
          label: segment.charAt(0).toUpperCase() + segment.slice(1).replace(/-/g, ' '),
          href: currentPath,
        });
      } else {
        breadcrumbs.push({
          label: segment.charAt(0).toUpperCase() + segment.slice(1).replace(/-/g, ' '),
          isCurrent: true,
        });
      }
    });

    return breadcrumbs;
  }

  return [];
}

function BreadcrumbItem({item, isLast}) {
  if (isLast || !item.href) {
    return (
      <li className={clsx('breadcrumb__item', styles.breadcrumbItem, isLast && styles.breadcrumbItemActive)}>
        <span className="breadcrumb__link breadcrumb__link--active">{item.label}</span>
      </li>
    );
  }

  return (
    <li className={clsx('breadcrumb__item', styles.breadcrumbItem)}>
      <Link to={item.href} className="breadcrumb__link">
        {item.label}
      </Link>
      <span className="breadcrumb__separator">»</span>
    </li>
  );
}

export default function Breadcrumb() {
  const breadcrumbs = useBreadcrumbs();

  if (breadcrumbs.length <= 1) {
    return null;
  }

  return (
    <nav className="breadcrumb" aria-label="breadcrumbs">
      <ul className={clsx('breadcrumb__list', styles.breadcrumbList)}>
        {breadcrumbs.map((item, idx) => (
          <BreadcrumbItem
            key={idx}
            item={item}
            isLast={idx === breadcrumbs.length - 1}
          />
        ))}
      </ul>
    </nav>
  );
}