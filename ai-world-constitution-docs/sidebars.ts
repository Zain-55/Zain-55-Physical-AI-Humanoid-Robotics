import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['introduction/what-is-physical-ai'],
    },
    {
      type: 'category',
      label: 'Module 1: ROS2',
      items: [
        'module-1-ros2/ros2-overview',
        'module-1-ros2/nodes-topics-services',
        'module-1-ros2/urdf-for-humanoids',
        'module-1-ros2/python-to-ros2-bridge',
    ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin',
      items: [
        'module-2-digital-twin/gazebo-basics',
        'module-2-digital-twin/physics-simulation',
        'module-2-digital-twin/unity-high-fidelity',
        'module-2-digital-twin/sensor-simulation',
    ],
    },
    {
        type: 'category',
        label: 'Module 3: NVIDIA Isaac',
        items: [
            'module-3-nvidia-isaac/isaac-sim-intro',
            'module-3-nvidia-isaac/perception-and-slam',
            'module-3-nvidia-isaac/navigation-nav2',
            'module-3-nvidia-isaac/sim-to-real-transfer',
        ],
    },
    {
        type: 'category',
        label: 'Module 4: VLA',
        items: [
            'module-4-vla/voice-to-action',
            'module-4-vla/cognitive-planning',
            'module-4-vla/multimodal-interaction',
            'module-4-vla/vla-integration',
        ],
    },
    {
        type: 'category',
        label: 'Capstone',
        items: ['capstone/autonomous-humanoid-project'],
    },
    {
        type: 'category',
        label: 'Hardware',
        items: [
            'hardware/workstation-requirements',
            'hardware/edge-ai-kits',
            'hardware/robot-lab-options',
        ],
    },
    {
        type: 'category',
        label: 'Weekly Breakdown',
        items: ['weekly-breakdown/weeks-1-to-13'],
    },
    {
        type: 'category',
        label: 'Assessments',
        items: [
            'assessments/ros2-project',
            'assessments/gazebo-simulation',
            'assessments/isaac-perception-pipeline',
            'assessments/capstone-evaluation',
        ],
    },
  ],
};

export default sidebars;