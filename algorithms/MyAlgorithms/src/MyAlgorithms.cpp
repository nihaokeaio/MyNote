#include "MyAlgorithms.h"
#include <vector>

int MyAlgorithms::serachStr(const string& txt, const string& pat)
{
	int i = 0, n = txt.size() - pat.size();
    while (i <= n)
    {
        if (txt[i] == pat[0])
        {
            int k = 0;
            while (k < pat.size())
            {
                if (txt[i + k] == pat[k])
                    ++k;
                else
                    break;
            }
            if (k == pat.size())
                return i;
        }
        ++i;
    }
    return -1;
}

int MyAlgorithms::serachStrKMP(const string& txt, const string& pat)
{
    int n = txt.size();
    int m = pat.size();
    vector<vector<int>>dp(m, vector<int>(256, 0));
    createKMPVec(dp, pat);

    int j = 0;
    for(int i=0;i<n;++i)
    {
        j = dp[j][txt[i]];
        if (j == m)
            return i - m + 1;
    }
    return -1;
}

void MyAlgorithms::createKMPVec(vector<vector<int>>& dp, const string& pat)
{
    int m = pat.size();
    int n = dp[0].size();

    //base status
    dp[0][pat[0]] = 1;
    int x = 0;

    ///这里c是字符的ASCII 码的数字化
    ///需要注意的是，这里从第一个状态开始开始匹配，i状态始终领先x状态，以此为状态机
    for (int i = 1; i < m; ++i)
    {
        for (int c = 0; c < n; ++c)
        {
            if (pat[i] == c)
            {
                dp[i][c] = i + 1;
            }
            else
            {
                dp[i][c] = dp[x][c];
            }
        }
        x = dp[x][pat[i]];
    }
}

ListNode* MyAlgorithms::reverseList(ListNode* head)
{
    ListNode* pre = nullptr;
    ListNode* cur = head;
    while(cur)
    {
        ListNode* tmp = cur->next;
        cur->next = pre;
        pre = cur;
        cur = tmp;
    }
    return pre;
}

ListNode* MyAlgorithms::removeNthFromEnd(ListNode* head, int n)
{
    ListNode* first = new ListNode(-1, head);
    ListNode* slow = first, * fast = first;
    while (n--)
    {
        fast = fast->next;
    }
    while (fast && fast->next)
    {
        fast = fast->next;
        slow = slow->next;
    }
    ListNode* p = slow->next;
    slow->next = p->next;
    delete p;
    return first->next;
}

void MyAlgorithms::traverse(TreeNode* root)
{
    if(root==nullptr)
        return;

    ///前序遍历
    traverse(root->left);
    ///中序遍历
    traverse(root->right);
    ///后续遍历
}
