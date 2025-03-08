using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class ExperimentRunner : MonoBehaviour
{
    [SerializeField] private GameObject agent;
    [SerializeField] private GameObject target;
    [SerializeField] private GameObject pid;
    [SerializeField] private int totalEpisodes = 100;
    [SerializeField] private string outputDirectory = "C:/Users/UserAdmin/Terrain Generation/Assets/DataProcessing";

    private List<float> ppoScores;
    private List<float> pidScores;
    private List<float> ppoMaxOutOfViewTimes; // Store max out-of-view times for PPO
    private List<float> pidMaxOutOfViewTimes; // Store max out-of-view times for PID

    private PathFollower pathFollower;
    private RL rlAgent;

    void Start()
    {
        File.WriteAllText($"{outputDirectory}/engagement_scores.csv", "Label,Min,Q1,Median,Q3,Max\n");
        File.WriteAllText($"{outputDirectory}/out_of_view_times.csv", "Label,Min,Q1,Median,Q3,Max\n");

        ppoScores = new List<float>();
        pidScores = new List<float>();
        ppoMaxOutOfViewTimes = new List<float>();
        pidMaxOutOfViewTimes = new List<float>();

        pathFollower = target.GetComponent<PathFollower>();
        rlAgent = agent.GetComponent<RL>();

        StartCoroutine(RunExperiment());
    }

    private IEnumerator RunExperiment()
    {
        for (int episodeIndex = 0; episodeIndex < totalEpisodes; episodeIndex++)
        {
            Debug.Log($"Running Episode {episodeIndex + 1}");

            ResetAgentsAndControllers();

            yield return new WaitUntil(() => pathFollower.lapCompleted);

            var agentScoreCalculator = agent.GetComponent<ScoreCalculator>();
            var pidScoreCalculator = pid.GetComponent<ScoreCalculator>();

            ppoScores.Add(agentScoreCalculator.percent);
            pidScores.Add(pidScoreCalculator.percent);

            ppoMaxOutOfViewTimes.Add(agentScoreCalculator.timePercent);
            pidMaxOutOfViewTimes.Add(pidScoreCalculator.timePercent);

            Debug.Log($"Episode {episodeIndex + 1} PPO score: {ppoScores[episodeIndex]}, PID score: {pidScores[episodeIndex]}");
            Debug.Log($"Episode {episodeIndex + 1} PPO Max Out-of-View Time: {ppoMaxOutOfViewTimes[episodeIndex]}, PID Max Out-of-View Time: {pidMaxOutOfViewTimes[episodeIndex]}");

            pathFollower.lapCompleted = false;
            agentScoreCalculator.ResetScore();
            pidScoreCalculator.ResetScore();
        }

        WriteSummaryToFile("PPO", ppoScores, $"{outputDirectory}/engagement_scores.csv");
        WriteSummaryToFile("PID", pidScores, $"{outputDirectory}/engagement_scores.csv");

        WriteSummaryToFile("PPO", ppoMaxOutOfViewTimes, $"{outputDirectory}/out_of_view_times.csv");
        WriteSummaryToFile("PID", pidMaxOutOfViewTimes, $"{outputDirectory}/out_of_view_times.csv");
    }

    private void ResetAgentsAndControllers()
    {
        pathFollower.Respawn();
        rlAgent.OnEpisodeBegin();
    }

    private void WriteSummaryToFile(string label, List<float> scores, string filePath)
    {
        scores.Sort();
        float min = scores[0];
        float max = scores[scores.Count - 1];
        float median = CalculateMedian(scores);
        float q1 = CalculateMedian(scores.GetRange(0, scores.Count / 2));
        float q3 = CalculateMedian(scores.GetRange((scores.Count + 1) / 2, scores.Count / 2));

        Debug.Log($"{label} - Min: {min}, Q1: {q1}, Median: {median}, Q3: {q3}, Max: {max}");
        File.AppendAllText(filePath, $"{label},{min},{q1},{median},{q3},{max}\n");
    }

    private float CalculateMedian(List<float> values)
    {
        int count = values.Count;
        if (count % 2 == 0)
            return (values[count / 2 - 1] + values[count / 2]) / 2f;
        else
            return values[count / 2];
    }
}
