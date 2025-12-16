// import React from 'react';
// import clsx from 'clsx';
// import Link from '@docusaurus/Link';
// import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
// import Layout from '@theme/Layout';
// import styles from './index.module.css';

// function HomepageHeader() {
//   const {siteConfig} = useDocusaurusContext();
//   return (
//     <header className={clsx('hero', styles.heroBanner)}>
//       <div className="container">
//         <div className={clsx('row', styles.heroRow)}>
          
//           {/* Left side - Image */}
//           <div className={clsx('col col--5', styles.heroLeft)}>
//             <div className={styles.imageContainer}>
//               <img 
//                 src="/robotik.jpg" 
//                 alt="Physical AI & Humanoid Robotics" 
//                 className={styles.heroImage}
//               />
//             </div>
//           </div>

//           {/* Right side - Text content */}
//           <div className={clsx('col col--7', styles.heroRight)}>
//             <h1 className={styles.heroTitle}>
//               Physical AI & Humanoid Robotics
//             </h1>
            
//             <p className={styles.heroSubtitle}>
//               Colearning Agentic AI with ROS 2, Gazebo and NVIDIA Isaac – Embodied Intelligence for the Physical World
//             </p>
            
//             <div className={styles.badgeContainer}>
//               <span className={styles.badge}>
//                 <span className={styles.badgeIcon}>⭐</span>
//                 Open Source
//               </span>
//               <span className={styles.badge}>
//                 <span className={styles.badgeIcon}>🧠</span>
//                 Co-Learning with AI
//               </span>
//               <span className={styles.badge}>
//                 <span className={styles.badgeIcon}>🎯</span>
//                 Spec-Driven Development
//               </span>
//             </div>
            
//             <div className={styles.buttons}>
//               <Link
//                 className={clsx('button button--lg', styles.btnOutline)}
//                 to="/">
//                 Explore More →
//               </Link>
//               <Link
//                 className={clsx('button button--lg', styles.btnSolid)}
//                 to="/docs/Book">
//                 Start Reading →
//               </Link>
//             </div>
//           </div>
          
//         </div>
//       </div>
//     </header>
//   );
// }

// export default function Home() {
//   const {siteConfig} = useDocusaurusContext();
//   return (
//     <Layout
//       title="Physical AI & Humanoid Robotics"
//       description="Master Physical AI & Humanoid Robotics with ROS 2, Gazebo, and NVIDIA Isaac">
//       <HomepageHeader />
//     </Layout>
//   );
// }




import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={clsx('row', styles.heroRow)}>
          
          {/* Left side - Image */}
          <div className={clsx('col col--5', styles.heroLeft)}>
            <div className={styles.imageContainer}>
              <img 
                src="/robotik.jpg" 
                alt="Physical AI & Humanoid Robotics" 
                className={styles.heroImage}
              />
            </div>
          </div>

          {/* Right side - Text content */}
          <div className={clsx('col col--7', styles.heroRight)}>
            <h1 className={styles.heroTitle}>
              Physical AI & Humanoid Robotics
            </h1>
            
            <p className={styles.heroSubtitle}>
              Colearning Agentic AI with ROS 2, Gazebo and NVIDIA Isaac – Embodied Intelligence for the Physical World
            </p>
            
            <div className={styles.badgeContainer}>
              <span className={styles.badge}>
                <span className={styles.badgeIcon}>⭐</span>
                Open Source
              </span>
              <span className={styles.badge}>
                <span className={styles.badgeIcon}>🧠</span>
                Co-Learning with AI
              </span>
              <span className={styles.badge}>
                <span className={styles.badgeIcon}>🎯</span>
                Spec-Driven Development
              </span>
            </div>
            
            <div className={styles.buttons}>
              <Link
                className={clsx('button button--lg', styles.btnOutline)}
                to="/">
                Explore More →
              </Link>
              <Link
                className={clsx('button button--lg', styles.btnSolid)}
                to="/docs">
                Start Reading →
              </Link>
            </div>
          </div>
          
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Master Physical AI & Humanoid Robotics with ROS 2, Gazebo, and NVIDIA Isaac">
      <HomepageHeader />
    </Layout>
  );
}