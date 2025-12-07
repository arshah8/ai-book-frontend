import Link from "next/link";
import Chatbot from "@/components/Chatbot";

export default function IntroPage() {
  return (
    <div className="min-h-screen bg-white dark:bg-gray-900">
      <div className="container mx-auto px-4 py-12 max-w-4xl">
        <Link
          href="/"
          className="text-indigo-600 hover:text-indigo-800 mb-8 inline-block"
        >
          ← Back to Home
        </Link>

        <article className="prose prose-lg dark:prose-invert max-w-none">
          <h1>Introduction to Physical AI & Humanoid Robotics</h1>

          <section className="my-8">
            <h2>Why Physical AI Matters</h2>
            <p>
              The future of AI extends beyond digital spaces into the physical world. 
              This course introduces Physical AI—AI systems that function in reality and 
              comprehend physical laws. Humanoid robots are poised to excel in our 
              human-centered world because they share our physical form and can be trained 
              with abundant data from interacting in human environments.
            </p>
            <p>
              This represents a significant transition from AI models confined to digital 
              environments to embodied intelligence that operates in physical space.
            </p>
          </section>

          <section className="my-8">
            <h2>Course Overview</h2>
            <p>
              This capstone quarter bridges the gap between the digital brain and the 
              physical body. Students learn to design, simulate, and deploy humanoid robots 
              capable of natural human interactions using ROS 2, Gazebo, and NVIDIA Isaac.
            </p>
          </section>

          <section className="my-8">
            <h2>Learning Outcomes</h2>
            <ul>
              <li>Understand Physical AI principles and embodied intelligence</li>
              <li>Master ROS 2 (Robot Operating System) for robotic control</li>
              <li>Simulate robots with Gazebo and Unity</li>
              <li>Develop with NVIDIA Isaac AI robot platform</li>
              <li>Design humanoid robots for natural interactions</li>
              <li>Integrate GPT models for conversational robotics</li>
            </ul>
          </section>

          <section className="my-8">
            <h2>Hardware Requirements</h2>
            <p>
              This course is technically demanding. It sits at the intersection of three 
              heavy computational loads: Physics Simulation (Isaac Sim/Gazebo), Visual 
              Perception (SLAM/Computer Vision), and Generative AI (LLMs/VLA).
            </p>
            <h3>Digital Twin Workstation (Required)</h3>
            <ul>
              <li><strong>GPU:</strong> NVIDIA RTX 4070 Ti (12GB VRAM) or higher</li>
              <li><strong>CPU:</strong> Intel Core i7 (13th Gen+) or AMD Ryzen 9</li>
              <li><strong>RAM:</strong> 64 GB DDR5 (32 GB minimum)</li>
              <li><strong>OS:</strong> Ubuntu 22.04 LTS</li>
            </ul>
            <h3>Physical AI Edge Kit</h3>
            <ul>
              <li><strong>Brain:</strong> NVIDIA Jetson Orin Nano (8GB) or Orin NX (16GB)</li>
              <li><strong>Vision:</strong> Intel RealSense D435i or D455</li>
              <li><strong>IMU:</strong> Generic USB IMU (BNO055)</li>
              <li><strong>Voice:</strong> USB Microphone/Speaker array</li>
            </ul>
          </section>

          <section className="my-8">
            <h2>Course Structure</h2>
            <p>
              The course is divided into four modules plus a capstone project:
            </p>
            <ol>
              <li><strong>Module 1:</strong> The Robotic Nervous System (ROS 2)</li>
              <li><strong>Module 2:</strong> The Digital Twin (Gazebo & Unity)</li>
              <li><strong>Module 3:</strong> The AI-Robot Brain (NVIDIA Isaac™)</li>
              <li><strong>Module 4:</strong> Vision-Language-Action (VLA)</li>
              <li><strong>Capstone:</strong> The Autonomous Humanoid</li>
            </ol>
          </section>
        </article>

        <div className="mt-12">
          <Link
            href="/module1"
            className="bg-indigo-600 text-white px-6 py-3 rounded-lg font-semibold hover:bg-indigo-700 transition-colors"
          >
            Start Module 1 →
          </Link>
        </div>
      </div>

      <Chatbot />
    </div>
  );
}

