import Link from "next/link";
import Chatbot from "@/components/Chatbot";

export default function Module3Page() {
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
          <h1>Module 3: The AI-Robot Brain (NVIDIA Isaac™)</h1>
          
          <section className="my-8">
            <h2>Introduction</h2>
            <p>
              NVIDIA Isaac is a comprehensive platform for developing AI-powered robots. 
              It provides tools for simulation, perception, manipulation, and reinforcement 
              learning, all optimized for NVIDIA GPUs.
            </p>
          </section>

          <section className="my-8">
            <h2>NVIDIA Isaac Sim</h2>
            <p>
              Isaac Sim is a photorealistic simulation environment built on NVIDIA Omniverse. 
              It provides:
            </p>
            <ul>
              <li>High-fidelity physics simulation</li>
              <li>Realistic rendering with ray tracing</li>
              <li>Synthetic data generation for training</li>
              <li>Domain randomization for sim-to-real transfer</li>
            </ul>
          </section>

          <section className="my-8">
            <h2>Isaac ROS</h2>
            <p>
              Isaac ROS provides hardware-accelerated ROS 2 packages optimized for NVIDIA 
              GPUs. Key capabilities include:
            </p>
            <ul>
              <li><strong>VSLAM (Visual SLAM):</strong> Real-time localization and mapping</li>
              <li><strong>Perception:</strong> Object detection and tracking</li>
              <li><strong>Navigation:</strong> Path planning and obstacle avoidance</li>
              <li><strong>Manipulation:</strong> Grasp planning and execution</li>
            </ul>
          </section>

          <section className="my-8">
            <h2>Nav2 for Bipedal Humanoid Movement</h2>
            <p>
              Nav2 is a navigation framework for ROS 2 that can be adapted for humanoid 
              robots. Unlike wheeled robots, humanoids require:
            </p>
            <ul>
              <li>Balance-aware path planning</li>
              <li>Footstep planning</li>
              <li>Dynamic stability considerations</li>
              <li>Adaptation to uneven terrain</li>
            </ul>
          </section>

          <section className="my-8">
            <h2>Reinforcement Learning for Robot Control</h2>
            <p>
              Isaac Sim includes tools for training reinforcement learning agents to control 
              robots. This enables:
            </p>
            <ul>
              <li>Learning complex behaviors through trial and error</li>
              <li>Sim-to-real transfer of learned policies</li>
              <li>Adaptive control strategies</li>
              <li>End-to-end learning from perception to action</li>
            </ul>
          </section>
        </article>

        <div className="mt-12 flex gap-4">
          <Link
            href="/module2"
            className="bg-gray-600 text-white px-6 py-3 rounded-lg font-semibold hover:bg-gray-700 transition-colors"
          >
            ← Previous
          </Link>
          <Link
            href="/module4"
            className="bg-indigo-600 text-white px-6 py-3 rounded-lg font-semibold hover:bg-indigo-700 transition-colors"
          >
            Next: Module 4 →
          </Link>
        </div>
      </div>

      <Chatbot />
    </div>
  );
}

