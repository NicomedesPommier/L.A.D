// src/levels/slidesROS2Concepts/08-TopicCommandsGame.jsx
import React from "react";

export const meta = {
  id: "topic-commands",
  title: "ROS 2 Topic Commands",
  order: 8,
  objectiveCode: "ros2-minigame-commands",
};

export default function TopicCommandsGame({ onObjectiveHit }) {
  const commands = [

    {
      command: "ros2 topic list",
      description: "List all active topics in the ROS 2 network",
      example: "$ ros2 topic list\n/chatter\n/parameter_events\n/rosout",
      useWhen: "You want to see what topics are available"
    },
    {
      command: "ros2 topic echo /topic_name",
      description: "Display messages being published on a topic in real-time",
      example: "$ ros2 topic echo /chatter\ndata: 'Hello ROS 2: 0'\n---\ndata: 'Hello ROS 2: 1'",
      useWhen: "You want to see what data is being published"
    },
    {
      command: "ros2 topic info /topic_name",
      description: "Show information about a topic (type, publishers, subscribers)",
      example: "$ ros2 topic info /chatter\nType: std_msgs/msg/String\nPublisher count: 1\nSubscription count: 1",
      useWhen: "You want to see who's using a topic"
    },
    {
      command: "ros2 topic hz /topic_name",
      description: "Measure the publishing rate (frequency) of a topic",
      example: "$ ros2 topic hz /chatter\naverage rate: 2.000\n\tmin: 0.500s max: 0.500s",
      useWhen: "You want to check how fast messages are being published"
    },
    {
      command: "ros2 topic pub /topic_name msg_type data",
      description: "Manually publish a message to a topic from the command line",
      example: "$ ros2 topic pub /chatter std_msgs/msg/String \"data: 'Hello from CLI'\"",
      useWhen: "You want to test a subscriber or send test data"
    },
  ];

  const [selected, setSelected] = React.useState(0);
  const [completed, setCompleted] = React.useState(new Set());

  const handleMarkComplete = (idx) => {
    const newCompleted = new Set(completed);
    newCompleted.add(idx);
    setCompleted(newCompleted);

    if (newCompleted.size === commands.length) {
      onObjectiveHit?.(meta.objectiveCode);
    }
  };

  return (
    <div className="slide">
      <h2>ROS 2 Topic Commands</h2>

      <div className="slide-card">
        <div className="slide-card__title">Essential CLI Tools</div>
        <p>
          ROS 2 provides powerful command-line tools to inspect and debug topics.
          Click through each command to learn what it does.
        </p>
        <p className="slide-text--sm slide-muted slide-mt-sm">
          Progress: {completed.size} / {commands.length} commands learned
        </p>
      </div>

      <div className="slide-grid slide-grid--30-70 slide-gap-md">
        {/* Command list */}
        <div className="slide-flex slide-flex--col slide-gap-sm">
          {commands.map((cmd, idx) => (
            <button
              key={idx}
              className={`btn btn--left-align ${selected === idx ? "btn--primary" : ""} ${completed.has(idx) ? "is-complete" : ""}`}
              onClick={() => setSelected(idx)}
              style={{ opacity: selected === idx ? 1 : 0.7 }}
            >
              {completed.has(idx) && <span className="slide-mr-sm">✓</span>}
              {cmd.command.split(" ")[2]} {/* Show just the command keyword like 'list', 'echo' */}
            </button>
          ))}
        </div>

        {/* Command details */}
        <div className="slide-card">
          <div className="slide-card__title">
            <code className="slide-text--neon">{commands[selected].command}</code>
          </div>

          <div className="slide-mt-md">
            <b>What it does:</b>
            <p className="slide-text--sm slide-muted">{commands[selected].description}</p>
          </div>

          <div className="slide-mt-md">
            <b>When to use it:</b>
            <p className="slide-text--sm slide-muted">{commands[selected].useWhen}</p>
          </div>

          <div className="slide-mt-md">
            <b>Example output:</b>
            <div className="slide-code-wrapper slide-mt-sm">
              <pre className="slide-code slide-code--sm">
                <code>{commands[selected].example}</code>
              </pre>
            </div>
          </div>

          <div className="slide-actions slide-mt-md">
            <button
              className="btn"
              onClick={() => navigator.clipboard?.writeText(commands[selected].command)}
            >
              Copy Command
            </button>
            {!completed.has(selected) && (
              <button
                className="btn btn--success"
                onClick={() => handleMarkComplete(selected)}
              >
                Mark as Learned
              </button>
            )}
          </div>
        </div>
      </div>

      {completed.size === commands.length && (
        <div className="slide-card slide-card--success slide-mt-md">
          <div className="slide-card__title">🎉 Congratulations!</div>
          <p>
            You've learned all the essential ROS 2 topic commands! These tools will be
            invaluable for debugging and understanding your ROS 2 systems.
          </p>
        </div>
      )}
    </div>
  );
}

