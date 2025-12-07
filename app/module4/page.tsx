import Link from "next/link";
import Chatbot from "@/components/Chatbot";

export default function Module4Page() {
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
          <h1>Module 4: Vision-Language-Action (VLA)</h1>
          
          <section className="my-8">
            <h2>Introduction</h2>
            <p>
              Vision-Language-Action (VLA) represents the convergence of Large Language 
              Models (LLMs) and Robotics. This module teaches how to use natural language 
              to control robots and enable conversational AI in robotic systems.
            </p>
          </section>

          <section className="my-8">
            <h2>Voice-to-Action: OpenAI Whisper</h2>
            <p>
              OpenAI Whisper is a speech recognition model that converts spoken language 
              into text. In robotics, this enables:
            </p>
            <ul>
              <li>Voice commands for robot control</li>
              <li>Natural language interaction</li>
              <li>Multi-language support</li>
              <li>Real-time speech transcription</li>
            </ul>
            <p>
              Integration with ROS 2 allows voice commands to be processed and converted 
              into robot actions.
            </p>
          </section>

          <section className="my-8">
            <h2>Cognitive Planning with LLMs</h2>
            <p>
              Large Language Models can translate high-level natural language commands 
              (like &quot;Clean the room&quot;) into sequences of robot actions. This involves:
            </p>
            <ul>
              <li><strong>Intent Understanding:</strong> Parsing user commands</li>
              <li><strong>Task Decomposition:</strong> Breaking down complex tasks</li>
              <li><strong>Action Planning:</strong> Generating ROS 2 action sequences</li>
              <li><strong>Error Recovery:</strong> Handling failures and replanning</li>
            </ul>
          </section>

          <section className="my-8">
            <h2>VLA Integration Architecture</h2>
            <p>
              A typical VLA system consists of:
            </p>
            <ol>
              <li><strong>Voice Input:</strong> Whisper transcribes speech to text</li>
              <li><strong>LLM Processing:</strong> GPT models understand intent and plan actions</li>
              <li><strong>Action Execution:</strong> ROS 2 nodes execute the planned actions</li>
              <li><strong>Feedback Loop:</strong> Vision and sensor data inform the LLM about progress</li>
            </ol>
          </section>

          <section className="my-8">
            <h2>Multi-Modal Interaction</h2>
            <p>
              Modern VLA systems combine multiple modalities:
            </p>
            <ul>
              <li><strong>Speech:</strong> Voice commands and responses</li>
              <li><strong>Vision:</strong> Understanding visual context</li>
              <li><strong>Gesture:</strong> Recognizing human gestures</li>
              <li><strong>Text:</strong> Chat-based interaction</li>
            </ul>
          </section>
        </article>

        <div className="mt-12 flex gap-4">
          <Link
            href="/module3"
            className="bg-gray-600 text-white px-6 py-3 rounded-lg font-semibold hover:bg-gray-700 transition-colors"
          >
            ← Previous
          </Link>
          <Link
            href="/capstone"
            className="bg-indigo-600 text-white px-6 py-3 rounded-lg font-semibold hover:bg-indigo-700 transition-colors"
          >
            Next: Capstone →
          </Link>
        </div>
      </div>

      <Chatbot />
    </div>
  );
}

