import Link from "next/link";
import Chatbot from "@/components/Chatbot";

export default function CapstonePage() {
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
          <h1>Capstone Project: The Autonomous Humanoid</h1>
          
          <section className="my-8">
            <h2>Project Overview</h2>
            <p>
              The capstone project brings together all the concepts learned throughout the 
              course. You will build a simulated humanoid robot that:
            </p>
            <ul>
              <li>Receives voice commands</li>
              <li>Plans a path to complete the task</li>
              <li>Navigates obstacles in the environment</li>
              <li>Identifies objects using computer vision</li>
              <li>Manipulates objects to complete the task</li>
            </ul>
          </section>

          <section className="my-8">
            <h2>System Architecture</h2>
            <p>
              Your autonomous humanoid system should integrate:
            </p>
            <ol>
              <li><strong>Voice Interface:</strong> Whisper for speech recognition</li>
              <li><strong>Planning Module:</strong> LLM-based task planning</li>
              <li><strong>Navigation Stack:</strong> Nav2 for path planning</li>
              <li><strong>Perception Pipeline:</strong> Object detection and recognition</li>
              <li><strong>Manipulation Controller:</strong> Grasp planning and execution</li>
            </ol>
          </section>

          <section className="my-8">
            <h2>Implementation Steps</h2>
            <h3>Phase 1: Setup and Simulation</h3>
            <ul>
              <li>Set up Gazebo/Isaac Sim environment</li>
              <li>Load humanoid robot model (URDF)</li>
              <li>Configure sensors (cameras, LiDAR, IMU)</li>
              <li>Test basic locomotion</li>
            </ul>

            <h3>Phase 2: Voice and Planning</h3>
            <ul>
              <li>Integrate Whisper for voice input</li>
              <li>Connect LLM for task planning</li>
              <li>Implement action sequence generation</li>
            </ul>

            <h3>Phase 3: Navigation</h3>
            <ul>
              <li>Configure Nav2 for humanoid navigation</li>
              <li>Implement obstacle avoidance</li>
              <li>Test path planning in various scenarios</li>
            </ul>

            <h3>Phase 4: Perception and Manipulation</h3>
            <ul>
              <li>Implement object detection pipeline</li>
              <li>Develop grasp planning algorithms</li>
              <li>Integrate manipulation controllers</li>
            </ul>

            <h3>Phase 5: Integration and Testing</h3>
            <ul>
              <li>End-to-end system integration</li>
              <li>Test complete task execution</li>
              <li>Optimize performance and reliability</li>
            </ul>
          </section>

          <section className="my-8">
            <h2>Evaluation Criteria</h2>
            <ul>
              <li>Successful voice command recognition</li>
              <li>Accurate task planning and decomposition</li>
              <li>Reliable navigation and obstacle avoidance</li>
              <li>Correct object identification</li>
              <li>Successful object manipulation</li>
              <li>System robustness and error handling</li>
            </ul>
          </section>

          <section className="my-8">
            <h2>Deliverables</h2>
            <ul>
              <li>Complete ROS 2 package with all nodes</li>
              <li>Simulation environment configuration</li>
              <li>Documentation of system architecture</li>
              <li>Demo video showing full task execution</li>
              <li>Code repository with clear README</li>
            </ul>
          </section>
        </article>

        <div className="mt-12">
          <Link
            href="/module4"
            className="bg-gray-600 text-white px-6 py-3 rounded-lg font-semibold hover:bg-gray-700 transition-colors"
          >
            ← Previous
          </Link>
        </div>
      </div>

      <Chatbot />
    </div>
  );
}

